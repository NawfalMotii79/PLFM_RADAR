"""
v7.workers — QThread-based workers and demo target simulator.

Classes:
  - RadarDataWorker  — reads from FT2232HQ, parses packets,
                       emits signals with processed data.
  - GPSDataWorker    — reads GPS frames from STM32 CDC, emits GPSData signals.
  - TargetSimulator  — QTimer-based demo target generator (from GUI_PyQt_Map.py).
"""

import logging
import math
import random
import time
from typing import List

from PyQt6.QtCore import QObject, QThread, QTimer, pyqtSignal

from .hardware import FT2232HQInterface, STM32USBInterface
from .models import GPSData, RadarSettings, RadarTarget
from .processing import (
    RadarPacketParser,
    RadarProcessor,
    USBPacketParser,
    apply_pitch_correction,
)

logger = logging.getLogger(__name__)


# =============================================================================
# Utility: polar → geographic
# =============================================================================

def polar_to_geographic(
    radar_lat: float,
    radar_lon: float,
    range_m: float,
    azimuth_deg: float,
) -> tuple:
    """
    Convert polar coordinates (range, azimuth) relative to radar
    to geographic (latitude, longitude).

    azimuth_deg: 0 = North, clockwise.
    Returns (lat, lon).
    """
    R = 6_371_000  # Earth radius in meters

    lat1 = math.radians(radar_lat)
    lon1 = math.radians(radar_lon)
    bearing = math.radians(azimuth_deg)

    lat2 = math.asin(
        math.sin(lat1) * math.cos(range_m / R)
        + math.cos(lat1) * math.sin(range_m / R) * math.cos(bearing)
    )
    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(range_m / R) * math.cos(lat1),
        math.cos(range_m / R) - math.sin(lat1) * math.sin(lat2),
    )
    return (math.degrees(lat2), math.degrees(lon2))


# =============================================================================
# Radar Data Worker (QThread)
# =============================================================================

class RadarDataWorker(QThread):
    """
    Background worker that continuously reads radar data from the primary
    FT2232HQ interface, parses packets, runs the processing pipeline, and
    emits signals with results.

    Signals:
        packetReceived(dict)     — a single parsed packet dict
        targetsUpdated(list)     — list of RadarTarget after processing
        errorOccurred(str)       — error message
        statsUpdated(dict)       — packet/byte counters
    """

    packetReceived = pyqtSignal(dict)
    targetsUpdated = pyqtSignal(list)
    errorOccurred = pyqtSignal(str)
    statsUpdated = pyqtSignal(dict)

    def __init__(
        self,
        ft2232hq: FT2232HQInterface,
        processor: RadarProcessor,
        packet_parser: RadarPacketParser,
        settings: RadarSettings,
        gps_data_ref: GPSData,
        parent=None,
    ):
        super().__init__(parent)
        self._ft2232hq = ft2232hq
        self._processor = processor
        self._parser = packet_parser
        self._settings = settings
        self._gps = gps_data_ref
        self._running = False

        # Counters
        self._packet_count = 0
        self._byte_count = 0
        self._error_count = 0

    def stop(self):
        self._running = False

    def run(self):
        """Main loop: read → parse → process → emit."""
        self._running = True
        buffer = bytearray()

        while self._running:
            # Use FT2232HQ interface
            iface = None
            if self._ft2232hq and self._ft2232hq.is_open:
                iface = self._ft2232hq

            if iface is None:
                self.msleep(100)
                continue

            try:
                data = iface.read_data(4096)
                if data:
                    buffer.extend(data)
                    self._byte_count += len(data)

                    # Parse as many packets as possible
                    while len(buffer) >= 6:
                        result = self._parser.parse_packet(bytes(buffer))
                        if result is None:
                            # No valid packet at current position — skip one byte
                            if len(buffer) > 1:
                                buffer = buffer[1:]
                            else:
                                break
                            continue

                        pkt, consumed = result
                        buffer = buffer[consumed:]
                        self._packet_count += 1

                        # Process the packet
                        self._process_packet(pkt)
                        self.packetReceived.emit(pkt)

                    # Emit stats periodically
                    self.statsUpdated.emit({
                        "packets": self._packet_count,
                        "bytes": self._byte_count,
                        "errors": self._error_count,
                        "active_tracks": len(self._processor.tracks),
                        "targets": len(self._processor.detected_targets),
                    })
                else:
                    self.msleep(10)
            except Exception as e:
                self._error_count += 1
                self.errorOccurred.emit(str(e))
                logger.error(f"RadarDataWorker error: {e}")
                self.msleep(100)

    # ---- internal packet handling ------------------------------------------

    def _process_packet(self, pkt: dict):
        """Route a parsed packet through the processing pipeline."""
        try:
            if pkt["type"] == "range":
                range_m = pkt["range"] * 0.1
                raw_elev = pkt["elevation"]
                corr_elev = apply_pitch_correction(raw_elev, self._gps.pitch)

                target = RadarTarget(
                    id=pkt["chirp"],
                    range=range_m,
                    velocity=0,
                    azimuth=pkt["azimuth"],
                    elevation=corr_elev,
                    snr=20.0,
                    timestamp=pkt["timestamp"],
                )
                self._update_rdm(target)

            elif pkt["type"] == "doppler":
                lam = 3e8 / self._settings.system_frequency
                velocity = (pkt["doppler_real"] / 32767.0) * (
                    self._settings.prf1 * lam / 2
                )
                self._update_velocity(pkt, velocity)

            elif pkt["type"] == "detection":
                if pkt["detected"]:
                    raw_elev = pkt["elevation"]
                    corr_elev = apply_pitch_correction(raw_elev, self._gps.pitch)
                    logger.info(
                        f"CFAR Detection: raw={raw_elev}, corr={corr_elev:.1f}, "
                        f"pitch={self._gps.pitch:.1f}"
                    )
        except Exception as e:
            logger.error(f"Error processing packet: {e}")

    def _update_rdm(self, target: RadarTarget):
        range_bin = min(int(target.range / 50), 1023)
        doppler_bin = min(abs(int(target.velocity)), 31)
        self._processor.range_doppler_map[range_bin, doppler_bin] += 1
        self._processor.detected_targets.append(target)
        if len(self._processor.detected_targets) > 100:
            self._processor.detected_targets = self._processor.detected_targets[-100:]

    def _update_velocity(self, pkt: dict, velocity: float):
        for t in self._processor.detected_targets:
            if (t.azimuth == pkt["azimuth"]
                    and t.elevation == pkt["elevation"]
                    and t.id == pkt["chirp"]):
                t.velocity = velocity
                break


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
            except Exception as e:
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
        self._targets: List[RadarTarget] = []
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
        range_m = random.uniform(5000, 40000)
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
        updated: List[RadarTarget] = []

        for t in self._targets:
            new_range = t.range - t.velocity * 0.5
            if new_range < 500 or new_range > 50000:
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
