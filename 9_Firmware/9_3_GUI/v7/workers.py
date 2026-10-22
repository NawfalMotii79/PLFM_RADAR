"""
v7.workers — QThread-based workers and demo target simulator.

Classes:
  - RadarDataWorker  — reads from FT2232H via production RadarAcquisition,
                       parses 0xAA/0xBB packets, assembles 64x32 frames,
                       runs host-side DSP, emits PyQt signals.
  - GPSDataWorker    — reads GPS frames from STM32 CDC, emits GPSData signals.
  - TargetSimulator  — QTimer-based demo target generator.

The old V6/V7 packet parsing (sync A5 C3 + type + CRC16) has been removed.
All packet parsing now uses the production radar_protocol.py which matches
the actual FPGA packet format (0xAA data 11-byte, 0xBB status 26-byte).
"""

import time
import random
import queue
import struct
import logging

import numpy as np

from PyQt6.QtCore import QThread, QObject, QTimer, pyqtSignal

from .models import RadarTarget, GPSData, RadarSettings, WaveformConfig
from .hardware import (
    RadarAcquisition,
    RadarFrame,
    StatusResponse,
    DataRecorder,
    STM32USBInterface,
)
from .processing import (
    RadarProcessor,
    USBPacketParser,
    apply_pitch_correction,
    polar_to_geographic,
)

logger = logging.getLogger(__name__)


# =============================================================================
# Radar Data Worker (QThread) — production protocol
# =============================================================================

class RadarDataWorker(QThread):
    """
    Background worker that reads radar data from FT2232H, parses 0xAA/0xBB
    packets via production RadarAcquisition, runs optional host-side DSP,
    and emits PyQt signals with results.

    Uses production radar_protocol.py for all packet parsing and frame
    assembly (11-byte 0xAA data packets → 64x32 RadarFrame).
    For replay, use ReplayWorker instead.

    Signals:
        frameReady(RadarFrame)    — a complete 64x32 radar frame
        statusReceived(object)    — StatusResponse from FPGA
        targetsUpdated(list)      — list of RadarTarget after host-side DSP
        errorOccurred(str)        — error message
        statsUpdated(dict)        — frame/byte counters
    """

    frameReady = pyqtSignal(object)       # RadarFrame
    statusReceived = pyqtSignal(object)   # StatusResponse
    targetsUpdated = pyqtSignal(list)     # List[RadarTarget]
    errorOccurred = pyqtSignal(str)
    statsUpdated = pyqtSignal(dict)

    def __init__(
        self,
        connection,  # FT2232HConnection
        processor: RadarProcessor | None = None,
        recorder: DataRecorder | None = None,
        gps_data_ref: GPSData | None = None,
        settings: RadarSettings | None = None,
        parent=None,
    ):
        super().__init__(parent)
        self._connection = connection
        self._processor = processor
        self._recorder = recorder
        self._gps = gps_data_ref
        self._settings = settings or RadarSettings()
        self._waveform = WaveformConfig()
        self._running = False

        # Frame queue for production RadarAcquisition → this thread
        self._frame_queue: queue.Queue = queue.Queue(maxsize=4)

        # Production acquisition thread (does the actual parsing)
        self._acquisition: RadarAcquisition | None = None

        # Counters
        self._frame_count = 0
        self._byte_count = 0
        self._error_count = 0

    def set_waveform(self, wf: "WaveformConfig") -> None:
        self._waveform = wf

    def stop(self):
        self._running = False
        if self._acquisition:
            self._acquisition.stop()

    def run(self):
        """
        Start production RadarAcquisition thread, then poll its frame queue
        and emit PyQt signals for each complete frame.
        """
        self._running = True

        # Create and start the production acquisition thread
        self._acquisition = RadarAcquisition(
            connection=self._connection,
            frame_queue=self._frame_queue,
            recorder=self._recorder,
            status_callback=self._on_status,
        )
        self._acquisition.start()
        logger.info("RadarDataWorker started (production protocol)")

        while self._running:
            try:
                # Poll for complete frames from production acquisition
                frame: RadarFrame = self._frame_queue.get(timeout=0.1)
                self._frame_count += 1

                # Emit raw frame
                self.frameReady.emit(frame)

                # Run host-side DSP if processor is configured
                if self._processor is not None:
                    targets = self._run_host_dsp(frame)
                    if targets:
                        self.targetsUpdated.emit(targets)

                # Emit stats
                self.statsUpdated.emit({
                    "frames": self._frame_count,
                    "detection_count": frame.detection_count,
                    "errors": self._error_count,
                })

            except queue.Empty:
                continue
            except (ValueError, IndexError) as e:
                self._error_count += 1
                self.errorOccurred.emit(str(e))
                logger.error(f"RadarDataWorker error: {e}")

        # Stop acquisition thread
        if self._acquisition:
            self._acquisition.stop()
            self._acquisition.join(timeout=2.0)
            self._acquisition = None

        logger.info("RadarDataWorker stopped")

    def _on_status(self, status: StatusResponse):
        """Callback from production RadarAcquisition on status packet."""
        self.statusReceived.emit(status)

    def _run_host_dsp(self, frame: RadarFrame) -> list[RadarTarget]:
        """
        Run host-side DSP on a complete frame.
        This is where DBSCAN clustering, Kalman tracking, and other
        non-timing-critical processing happens.

        The FPGA already does: FFT, MTI, CFAR, DC notch.
        Host-side DSP adds: clustering, tracking, geo-coordinate mapping.

        Bin-to-physical conversion uses self._waveform (WaveformConfig) to keep
        live and replay units aligned. Override via set_waveform() if needed.
        """
        targets: list[RadarTarget] = []

        cfg = self._processor.config
        if not (cfg.clustering_enabled or cfg.tracking_enabled):
            return targets

        # Extract detections from FPGA CFAR flags
        det_indices = np.argwhere(frame.detections > 0)
        r_res = self._waveform.range_resolution_m
        v_res = self._waveform.velocity_resolution_mps
        n_doppler = (frame.detections.shape[1] if frame.detections.ndim == 2
                     else self._waveform.n_doppler_bins)
        doppler_center = n_doppler // 2

        for idx in det_indices:
            rbin, dbin = idx
            mag = frame.magnitude[rbin, dbin]
            snr = 10 * np.log10(max(mag, 1)) if mag > 0 else 0

            # Convert bin indices to physical units
            range_m = float(rbin) * r_res
            # Doppler: centre bin = 0 m/s; positive bins = approaching.
            # Derived from frame shape — mirrors processing.py:520.
            velocity_ms = float(dbin - doppler_center) * v_res

            # Apply pitch correction if GPS data available
            raw_elev = 0.0  # FPGA doesn't send elevation per-detection
            corr_elev = raw_elev
            if self._gps:
                corr_elev = apply_pitch_correction(raw_elev, self._gps.pitch)

            # Compute geographic position if GPS available
            lat, lon = 0.0, 0.0
            azimuth = 0.0  # No azimuth from single-beam; set to heading
            if self._gps:
                azimuth = self._gps.heading
                lat, lon = polar_to_geographic(
                    self._gps.latitude, self._gps.longitude,
                    range_m, azimuth,
                )

            target = RadarTarget(
                id=len(targets),
                range=range_m,
                velocity=velocity_ms,
                azimuth=azimuth,
                elevation=corr_elev,
                latitude=lat,
                longitude=lon,
                snr=snr,
                timestamp=frame.timestamp,
            )
            targets.append(target)

        # DBSCAN clustering
        if cfg.clustering_enabled and len(targets) > 0:
            clusters = self._processor.clustering(
                targets, cfg.clustering_eps, cfg.clustering_min_samples)
            # Associate and track
            if cfg.tracking_enabled:
                targets = self._processor.association(targets, clusters)
                self._processor.tracking(targets)

        return targets


# =============================================================================
# GPS Data Worker (QThread)
# =============================================================================

class GPSDataWorker(QThread):
    """
    Background worker that reads GPS frames from the STM32 USB CDC interface
    and emits parsed GPSData objects.

    Signals:
        gpsReceived(GPSData)
        errorOccurred(str)
    """

    gpsReceived = pyqtSignal(object)   # GPSData
    errorOccurred = pyqtSignal(str)

    def __init__(
        self,
        stm32: STM32USBInterface,
        usb_parser: USBPacketParser,
        parent=None,
    ):
        super().__init__(parent)
        self._stm32 = stm32
        self._parser = usb_parser
        self._running = False
        self._gps_count = 0

    @property
    def gps_count(self) -> int:
        return self._gps_count

    def stop(self):
        self._running = False

    def run(self):
        self._running = True
        while self._running:
            if not (self._stm32 and self._stm32.is_open):
                self.msleep(100)
                continue

            try:
                data = self._stm32.read_data(64, timeout=100)
                if data:
                    gps = self._parser.parse_gps_data(data)
                    if gps:
                        self._gps_count += 1
                        self.gpsReceived.emit(gps)
            except (ValueError, struct.error) as e:
                self.errorOccurred.emit(str(e))
                logger.error(f"GPSDataWorker error: {e}")
            self.msleep(100)


# =============================================================================
# Target Simulator (Demo Mode) — QTimer-based
# =============================================================================

class TargetSimulator(QObject):
    """
    Generates simulated radar targets for demo/testing.

    Uses a QTimer on the main thread (or whichever thread owns this object).
    Emits ``targetsUpdated`` with a list[RadarTarget] on each tick.
    """

    targetsUpdated = pyqtSignal(list)

    def __init__(self, radar_position: GPSData, parent=None):
        super().__init__(parent)
        self._radar_pos = radar_position
        self._targets: list[RadarTarget] = []
        self._next_id = 1
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._initialize_targets(8)

    # ---- public API --------------------------------------------------------

    def start(self, interval_ms: int = 500):
        self._timer.start(interval_ms)

    def stop(self):
        self._timer.stop()

    def set_radar_position(self, gps: GPSData):
        self._radar_pos = gps

    def add_random_target(self):
        self._add_random_target()

    # ---- internals ---------------------------------------------------------

    def _initialize_targets(self, count: int):
        for _ in range(count):
            self._add_random_target()

    def _add_random_target(self):
        range_m = random.uniform(50, 1400)
        azimuth = random.uniform(0, 360)
        velocity = random.uniform(-100, 100)
        elevation = random.uniform(-5, 45)

        lat, lon = polar_to_geographic(
            self._radar_pos.latitude,
            self._radar_pos.longitude,
            range_m,
            azimuth,
        )

        target = RadarTarget(
            id=self._next_id,
            range=range_m,
            velocity=velocity,
            azimuth=azimuth,
            elevation=elevation,
            latitude=lat,
            longitude=lon,
            snr=random.uniform(10, 35),
            timestamp=time.time(),
            track_id=self._next_id,
            classification=random.choice(["aircraft", "drone", "bird", "unknown"]),
        )
        self._next_id += 1
        self._targets.append(target)

    def _tick(self):
        """Update all simulated targets and emit."""
        updated: list[RadarTarget] = []

        for t in self._targets:
            new_range = t.range - t.velocity * 0.5
            if new_range < 10 or new_range > 1536:
                continue  # target exits coverage — drop it

            new_vel = max(-150, min(150, t.velocity + random.uniform(-2, 2)))
            new_az = (t.azimuth + random.uniform(-0.5, 0.5)) % 360

            lat, lon = polar_to_geographic(
                self._radar_pos.latitude,
                self._radar_pos.longitude,
                new_range,
                new_az,
            )

            updated.append(RadarTarget(
                id=t.id,
                range=new_range,
                velocity=new_vel,
                azimuth=new_az,
                elevation=t.elevation + random.uniform(-0.1, 0.1),
                latitude=lat,
                longitude=lon,
                snr=t.snr + random.uniform(-1, 1),
                timestamp=time.time(),
                track_id=t.track_id,
                classification=t.classification,
            ))

        # Maintain a reasonable target count
        if len(updated) < 5 or (random.random() < 0.05 and len(updated) < 15):
            self._add_random_target()
            updated.append(self._targets[-1])

        self._targets = updated
        self.targetsUpdated.emit(updated)


# =============================================================================
# Replay Worker (QThread) — unified replay playback
# =============================================================================

class ReplayWorker(QThread):
    """Background worker for replay data playback.

    Emits the same signals as ``RadarDataWorker`` so the dashboard
    treats live and replay identically.  Additionally emits playback
    state and frame-index signals for the transport controls.

    Signals
    -------
    frameReady(object)           RadarFrame
    targetsUpdated(list)         list[RadarTarget]
    statsUpdated(dict)           processing stats
    errorOccurred(str)           error message
    playbackStateChanged(str)    "playing" | "paused" | "stopped"
    frameIndexChanged(int, int)  (current_index, total_frames)
    """

    frameReady = pyqtSignal(object)
    targetsUpdated = pyqtSignal(list)
    statsUpdated = pyqtSignal(dict)
    errorOccurred = pyqtSignal(str)
    playbackStateChanged = pyqtSignal(str)
    frameIndexChanged = pyqtSignal(int, int)

    def __init__(
        self,
        replay_engine,
        settings: RadarSettings | None = None,
        gps: GPSData | None = None,
        frame_interval_ms: int = 100,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        import threading

        from .processing import extract_targets_from_frame
        from .models import WaveformConfig

        self._engine = replay_engine
        self._settings = settings or RadarSettings()
        self._gps = gps
        self._waveform = WaveformConfig()
        self._frame_interval_ms = frame_interval_ms
        self._extract_targets = extract_targets_from_frame

        self._current_index = 0
        self._last_emitted_index = 0
        self._playing = False
        self._stop_flag = False
        self._loop = False
        self._lock = threading.Lock()  # guards _current_index and _emit_frame

    # -- Public control API --

    @property
    def current_index(self) -> int:
        """Index of the last frame emitted (for re-seek on param change)."""
        return self._last_emitted_index

    @property
    def total_frames(self) -> int:
        return self._engine.total_frames

    def set_gps(self, gps: GPSData | None) -> None:
        self._gps = gps

    def set_waveform(self, wf) -> None:
        self._waveform = wf

    def set_loop(self, loop: bool) -> None:
        self._loop = loop

    def set_frame_interval(self, ms: int) -> None:
        self._frame_interval_ms = max(10, ms)

    def play(self) -> None:
        self._playing = True
        # If at EOF, rewind so play actually does something
        with self._lock:
            if self._current_index >= self._engine.total_frames:
                self._current_index = 0
        self.playbackStateChanged.emit("playing")

    def pause(self) -> None:
        self._playing = False
        self.playbackStateChanged.emit("paused")

    def stop(self) -> None:
        self._playing = False
        self._stop_flag = True
        self.playbackStateChanged.emit("stopped")

    @property
    def is_playing(self) -> bool:
        """Thread-safe read of playback state (for GUI queries)."""
        return self._playing

    def seek(self, index: int) -> None:
        """Jump to a specific frame and emit it (thread-safe)."""
        with self._lock:
            idx = max(0, min(index, self._engine.total_frames - 1))
            self._current_index = idx
            self._emit_frame(idx)
            self._last_emitted_index = idx

    # -- Thread entry --

    def run(self) -> None:
        self._stop_flag = False
        self._playing = True
        self.playbackStateChanged.emit("playing")

        try:
            while not self._stop_flag:
                if self._playing:
                    with self._lock:
                        if self._current_index < self._engine.total_frames:
                            self._emit_frame(self._current_index)
                            self._last_emitted_index = self._current_index
                            self._current_index += 1

                            # Loop or pause at end
                            if self._current_index >= self._engine.total_frames:
                                if self._loop:
                                    self._current_index = 0
                                else:
                                    # Pause — keep thread alive for restart
                                    self._playing = False
                                    self.playbackStateChanged.emit("stopped")

                self.msleep(self._frame_interval_ms)
        except (OSError, ValueError, RuntimeError, IndexError) as exc:
            self.errorOccurred.emit(str(exc))

        self.playbackStateChanged.emit("stopped")

    # -- Internal --

    def _emit_frame(self, index: int) -> None:
        try:
            frame = self._engine.get_frame(index)
        except (OSError, ValueError, RuntimeError, IndexError) as exc:
            self.errorOccurred.emit(f"Frame {index}: {exc}")
            return

        self.frameReady.emit(frame)
        self.frameIndexChanged.emit(index, self._engine.total_frames)

        # Target extraction
        targets = self._extract_targets(
            frame,
            range_resolution=self._waveform.range_resolution_m,
            velocity_resolution=self._waveform.velocity_resolution_mps,
            gps=self._gps,
        )
        self.targetsUpdated.emit(targets)
        self.statsUpdated.emit({
            "frame_number": frame.frame_number,
            "detection_count": frame.detection_count,
            "target_count": len(targets),
            "replay_index": index,
            "replay_total": self._engine.total_frames,
        })
