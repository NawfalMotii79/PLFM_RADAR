"""
v7.processing — Radar signal processing, packet parsing, and GPS parsing.

Classes:
  - RadarProcessor   — dual-CPI fusion, multi-PRF unwrap, DBSCAN clustering,
                        association, Kalman tracking
  - RadarPacketParser — parse raw byte streams into typed radar packets
                        (FIX: returns (parsed_dict, bytes_consumed) tuple)
  - USBPacketParser   — parse GPS text/binary frames from STM32 CDC

Bug fixes vs V6:
  1. RadarPacketParser.parse_packet() now returns (dict, bytes_consumed) tuple
     so the caller knows exactly how many bytes to strip from the buffer.
  2. apply_pitch_correction() is a proper standalone function.
"""

import logging
import math
import struct
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

from .models import (
    CRCMOD_AVAILABLE,
    FILTERPY_AVAILABLE,
    SCIPY_AVAILABLE,
    SKLEARN_AVAILABLE,
    GPSData,
    ProcessingConfig,
    RadarTarget,
)

if SKLEARN_AVAILABLE:
    from sklearn.cluster import DBSCAN

if FILTERPY_AVAILABLE:
    from filterpy.kalman import KalmanFilter

if CRCMOD_AVAILABLE:
    import crcmod

if SCIPY_AVAILABLE:
    from scipy.signal import windows as scipy_windows

logger = logging.getLogger(__name__)


# =============================================================================
# Utility: pitch correction (Bug #4 fix — was never defined in V6)
# =============================================================================

def apply_pitch_correction(raw_elevation: float, pitch: float) -> float:
    """
    Apply platform pitch correction to a raw elevation angle.

    Returns the corrected elevation = raw_elevation - pitch.
    """
    return raw_elevation - pitch


# =============================================================================
# Radar Processor — signal-level processing & tracking pipeline
# =============================================================================

class RadarProcessor:
    """Full radar processing pipeline: fusion, clustering, association, tracking."""

    def __init__(self):
        self.range_doppler_map = np.zeros((1024, 32))
        self.detected_targets: List[RadarTarget] = []
        self.track_id_counter: int = 0
        self.tracks: Dict[int, dict] = {}
        self.frame_count: int = 0
        self.config = ProcessingConfig()

        # MTI state: store previous frames for cancellation
        self._mti_history: List[np.ndarray] = []

    # ---- Configuration -----------------------------------------------------

    def set_config(self, config: ProcessingConfig):
        """Update the processing configuration and reset MTI history if needed."""
        old_order = self.config.mti_order
        self.config = config
        if config.mti_order != old_order:
            self._mti_history.clear()

    # ---- Windowing ----------------------------------------------------------

    @staticmethod
    def apply_window(data: np.ndarray, window_type: str) -> np.ndarray:
        """Apply a window function along each column (slow-time dimension).

        *data* shape: (range_bins, doppler_bins).  Window is applied along
        axis-1 (Doppler / slow-time).
        """
        if window_type == "None" or not window_type:
            return data

        n = data.shape[1]
        if n < 2:
            return data

        if SCIPY_AVAILABLE:
            wtype = window_type.lower()
            if wtype == "hann":
                w = scipy_windows.hann(n, sym=False)
            elif wtype == "hamming":
                w = scipy_windows.hamming(n, sym=False)
            elif wtype == "blackman":
                w = scipy_windows.blackman(n)
            elif wtype == "kaiser":
                w = scipy_windows.kaiser(n, beta=14)
            elif wtype == "chebyshev":
                w = scipy_windows.chebwin(n, at=80)
            else:
                w = np.ones(n)
        else:
            # Fallback: numpy Hann
            wtype = window_type.lower()
            if wtype == "hann":
                w = np.hanning(n)
            elif wtype == "hamming":
                w = np.hamming(n)
            elif wtype == "blackman":
                w = np.blackman(n)
            else:
                w = np.ones(n)

        return data * w[np.newaxis, :]

    # ---- DC Notch (zero-Doppler removal) ------------------------------------

    @staticmethod
    def dc_notch(data: np.ndarray) -> np.ndarray:
        """Remove the DC (zero-Doppler) component by subtracting the
        mean along the slow-time axis for each range bin."""
        return data - np.mean(data, axis=1, keepdims=True)

    # ---- MTI (Moving Target Indication) -------------------------------------

    def mti_filter(self, frame: np.ndarray) -> np.ndarray:
        """Apply MTI cancellation of order 1, 2, or 3.

        Order-1: y[n] = x[n] - x[n-1]
        Order-2: y[n] = x[n] - 2*x[n-1] + x[n-2]
        Order-3: y[n] = x[n] - 3*x[n-1] + 3*x[n-2] - x[n-3]

        The internal history buffer stores up to 3 previous frames.
        """
        order = self.config.mti_order
        self._mti_history.append(frame.copy())

        # Trim history to order + 1 frames
        max_len = order + 1
        if len(self._mti_history) > max_len:
            self._mti_history = self._mti_history[-max_len:]

        if len(self._mti_history) < order + 1:
            # Not enough history yet — return zeros (suppress output)
            return np.zeros_like(frame)

        h = self._mti_history
        if order == 1:
            return h[-1] - h[-2]
        elif order == 2:
            return h[-1] - 2.0 * h[-2] + h[-3]
        elif order == 3:
            return h[-1] - 3.0 * h[-2] + 3.0 * h[-3] - h[-4]
        else:
            return h[-1] - h[-2]

    # ---- CFAR (Constant False Alarm Rate) -----------------------------------

    @staticmethod
    def cfar_1d(signal_vec: np.ndarray, guard: int, train: int,
                threshold_factor: float, cfar_type: str = "CA-CFAR") -> np.ndarray:
        """1-D CFAR detector.

        Parameters
        ----------
        signal_vec : 1-D array (power in linear scale)
        guard      : number of guard cells on each side
        train      : number of training cells on each side
        threshold_factor : multiplier on estimated noise level
        cfar_type  : CA-CFAR, OS-CFAR, GO-CFAR, or SO-CFAR

        Returns
        -------
        detections : boolean array, True where target detected
        """
        n = len(signal_vec)
        detections = np.zeros(n, dtype=bool)
        half = guard + train

        for i in range(half, n - half):
            # Leading training cells
            lead = signal_vec[i - half: i - guard]
            # Lagging training cells
            lag = signal_vec[i + guard + 1: i + half + 1]

            if cfar_type == "CA-CFAR":
                noise = (np.sum(lead) + np.sum(lag)) / (2 * train)
            elif cfar_type == "GO-CFAR":
                noise = max(np.mean(lead), np.mean(lag))
            elif cfar_type == "SO-CFAR":
                noise = min(np.mean(lead), np.mean(lag))
            elif cfar_type == "OS-CFAR":
                all_train = np.concatenate([lead, lag])
                all_train.sort()
                k = int(0.75 * len(all_train))  # 75th percentile
                noise = all_train[min(k, len(all_train) - 1)]
            else:
                noise = (np.sum(lead) + np.sum(lag)) / (2 * train)

            threshold = noise * threshold_factor
            if signal_vec[i] > threshold:
                detections[i] = True

        return detections

    def cfar_2d(self, rdm: np.ndarray) -> np.ndarray:
        """Apply 1-D CFAR along each range bin (across Doppler dimension).

        Returns a boolean mask of the same shape as *rdm*.
        """
        cfg = self.config
        mask = np.zeros_like(rdm, dtype=bool)
        for r in range(rdm.shape[0]):
            row = rdm[r, :]
            if row.max() > 0:
                mask[r, :] = self.cfar_1d(
                    row, cfg.cfar_guard_cells, cfg.cfar_training_cells,
                    cfg.cfar_threshold_factor, cfg.cfar_type,
                )
        return mask

    # ---- Full processing pipeline -------------------------------------------

    def process_frame(self, raw_frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Run the full signal processing chain on a Range x Doppler frame.

        Parameters
        ----------
        raw_frame : 2-D array (range_bins x doppler_bins), complex or real

        Returns
        -------
        (processed_rdm, detection_mask)
            processed_rdm  — processed Range-Doppler map (power, linear)
            detection_mask — boolean mask of CFAR / threshold detections
        """
        cfg = self.config
        data = raw_frame.astype(np.float64)

        # 1. DC Notch
        if cfg.dc_notch_enabled:
            data = self.dc_notch(data)

        # 2. Windowing (before FFT — applied along slow-time axis)
        if cfg.window_type and cfg.window_type != "None":
            data = self.apply_window(data, cfg.window_type)

        # 3. MTI
        if cfg.mti_enabled:
            data = self.mti_filter(data)

        # 4. Power (magnitude squared)
        power = np.abs(data) ** 2
        power = np.maximum(power, 1e-20)  # avoid log(0)

        # 5. CFAR detection or simple threshold
        if cfg.cfar_enabled:
            detection_mask = self.cfar_2d(power)
        else:
            # Simple threshold: convert dB threshold to linear
            power_db = 10.0 * np.log10(power)
            noise_floor = np.median(power_db)
            detection_mask = power_db > (noise_floor + cfg.detection_threshold_db)

        # Update stored RDM
        self.range_doppler_map = power
        self.frame_count += 1

        return power, detection_mask

    # ---- Dual-CPI fusion ---------------------------------------------------

    @staticmethod
    def dual_cpi_fusion(range_profiles_1: np.ndarray,
                        range_profiles_2: np.ndarray) -> np.ndarray:
        """Dual-CPI fusion for better detection."""
        return np.mean(range_profiles_1, axis=0) + np.mean(range_profiles_2, axis=0)

    # ---- Multi-PRF velocity unwrapping -------------------------------------

    def multi_prf_unwrap(self, doppler_measurements, prf1: float, prf2: float):
        """Multi-PRF velocity unwrapping (Chinese Remainder Theorem)."""
        lam = 3e8 / 10e9
        v_max1 = prf1 * lam / 2
        v_max2 = prf2 * lam / 2

        unwrapped = []
        for doppler in doppler_measurements:
            v1 = doppler * lam / 2
            v2 = doppler * lam / 2
            velocity = self._solve_chinese_remainder(v1, v2, v_max1, v_max2)
            unwrapped.append(velocity)
        return unwrapped

    @staticmethod
    def _solve_chinese_remainder(v1, v2, max1, max2):
        for k in range(-5, 6):
            candidate = v1 + k * max1
            if abs(candidate - v2) < max2 / 2:
                return candidate
        return v1

    # ---- DBSCAN clustering -------------------------------------------------

    @staticmethod
    def clustering(detections: List[RadarTarget],
                   eps: float = 100, min_samples: int = 2) -> list:
        """DBSCAN clustering of detections (requires sklearn)."""
        if not SKLEARN_AVAILABLE or len(detections) == 0:
            return []

        points = np.array([[d.range, d.velocity] for d in detections])
        labels = DBSCAN(eps=eps, min_samples=min_samples).fit(points).labels_

        clusters = []
        for label in set(labels):
            if label == -1:
                continue
            cluster_points = points[labels == label]
            clusters.append({
                "center": np.mean(cluster_points, axis=0),
                "points": cluster_points,
                "size": len(cluster_points),
            })
        return clusters

    # ---- Association -------------------------------------------------------

    def association(self, detections: List[RadarTarget],
                    clusters: list) -> List[RadarTarget]:
        """Associate detections to existing tracks (nearest-neighbour)."""
        associated = []
        for det in detections:
            best_track = None
            min_dist = float("inf")
            for tid, track in self.tracks.items():
                dist = math.sqrt(
                    (det.range - track["state"][0]) ** 2
                    + (det.velocity - track["state"][2]) ** 2
                )
                if dist < min_dist and dist < 500:
                    min_dist = dist
                    best_track = tid

            if best_track is not None:
                det.track_id = best_track
            else:
                det.track_id = self.track_id_counter
                self.track_id_counter += 1

            associated.append(det)
        return associated

    # ---- Kalman tracking ---------------------------------------------------

    def tracking(self, associated_detections: List[RadarTarget]):
        """Kalman filter tracking (requires filterpy)."""
        if not FILTERPY_AVAILABLE:
            return

        now = time.time()

        for det in associated_detections:
            if det.track_id not in self.tracks:
                kf = KalmanFilter(dim_x=4, dim_z=2)
                kf.x = np.array([det.range, 0, det.velocity, 0])
                kf.F = np.array([
                    [1, 1, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 1],
                    [0, 0, 0, 1],
                ])
                kf.H = np.array([
                    [1, 0, 0, 0],
                    [0, 0, 1, 0],
                ])
                kf.P *= 1000
                kf.R = np.diag([10, 1])
                kf.Q = np.eye(4) * 0.1

                self.tracks[det.track_id] = {
                    "filter": kf,
                    "state": kf.x,
                    "last_update": now,
                    "hits": 1,
                }
            else:
                track = self.tracks[det.track_id]
                track["filter"].predict()
                track["filter"].update([det.range, det.velocity])
                track["state"] = track["filter"].x
                track["last_update"] = now
                track["hits"] += 1

        # Prune stale tracks (> 5 s without update)
        stale = [tid for tid, t in self.tracks.items()
                 if now - t["last_update"] > 5.0]
        for tid in stale:
            del self.tracks[tid]


# =============================================================================
# Radar Packet Parser
# =============================================================================

class RadarPacketParser:
    """
    Parse binary radar packets from the raw byte stream.

    Packet format:
        [Sync 2][Type 1][Length 1][Payload N][CRC16 2]
    Sync pattern: 0xA5 0xC3

    Bug fix vs V6:
        parse_packet() now returns ``(parsed_dict, bytes_consumed)`` so the
        caller can correctly advance the read pointer in the buffer.
    """

    SYNC = b"\xA5\xC3"

    def __init__(self):
        if CRCMOD_AVAILABLE:
            self.crc16_func = crcmod.mkCrcFun(
                0x11021, rev=False, initCrc=0xFFFF, xorOut=0x0000
            )
        else:
            self.crc16_func = None

    # ---- main entry point --------------------------------------------------

    def parse_packet(self, data: bytes) -> Optional[Tuple[dict, int]]:
        """
        Attempt to parse one radar packet from *data*.

        Returns
        -------
        (parsed_dict, bytes_consumed) on success, or None if no valid packet.
        """
        if len(data) < 6:
            return None

        idx = data.find(self.SYNC)
        if idx == -1:
            return None

        pkt = data[idx:]
        if len(pkt) < 6:
            return None

        pkt_type = pkt[2]
        length = pkt[3]
        total_len = 4 + length + 2   # sync(2) + type(1) + len(1) + payload + crc(2)

        if len(pkt) < total_len:
            return None

        payload = pkt[4 : 4 + length]
        crc_received = struct.unpack("<H", pkt[4 + length : 4 + length + 2])[0]

        # CRC check
        if self.crc16_func is not None:
            crc_calc = self.crc16_func(pkt[0 : 4 + length])
            if crc_calc != crc_received:
                logger.warning(
                    f"CRC mismatch: got {crc_received:04X}, calc {crc_calc:04X}"
                )
                return None

        # Bytes consumed = offset to sync + total packet length
        consumed = idx + total_len

        parsed = None
        if pkt_type == 0x01:
            parsed = self._parse_range(payload)
        elif pkt_type == 0x02:
            parsed = self._parse_doppler(payload)
        elif pkt_type == 0x03:
            parsed = self._parse_detection(payload)
        else:
            logger.warning(f"Unknown packet type: {pkt_type:02X}")

        if parsed is None:
            return None
        return (parsed, consumed)

    # ---- sub-parsers -------------------------------------------------------

    @staticmethod
    def _parse_range(payload: bytes) -> Optional[dict]:
        if len(payload) < 12:
            return None
        try:
            range_val = struct.unpack(">I", payload[0:4])[0]
            elevation = payload[4] & 0x1F
            azimuth = payload[5] & 0x3F
            chirp = payload[6] & 0x1F
            return {
                "type": "range",
                "range": range_val,
                "elevation": elevation,
                "azimuth": azimuth,
                "chirp": chirp,
                "timestamp": time.time(),
            }
        except Exception as e:
            logger.error(f"Error parsing range packet: {e}")
            return None

    @staticmethod
    def _parse_doppler(payload: bytes) -> Optional[dict]:
        if len(payload) < 12:
            return None
        try:
            real = struct.unpack(">h", payload[0:2])[0]
            imag = struct.unpack(">h", payload[2:4])[0]
            elevation = payload[4] & 0x1F
            azimuth = payload[5] & 0x3F
            chirp = payload[6] & 0x1F
            return {
                "type": "doppler",
                "doppler_real": real,
                "doppler_imag": imag,
                "elevation": elevation,
                "azimuth": azimuth,
                "chirp": chirp,
                "timestamp": time.time(),
            }
        except Exception as e:
            logger.error(f"Error parsing doppler packet: {e}")
            return None

    @staticmethod
    def _parse_detection(payload: bytes) -> Optional[dict]:
        if len(payload) < 8:
            return None
        try:
            detected = (payload[0] & 0x01) != 0
            elevation = payload[1] & 0x1F
            azimuth = payload[2] & 0x3F
            chirp = payload[3] & 0x1F
            return {
                "type": "detection",
                "detected": detected,
                "elevation": elevation,
                "azimuth": azimuth,
                "chirp": chirp,
                "timestamp": time.time(),
            }
        except Exception as e:
            logger.error(f"Error parsing detection packet: {e}")
            return None


# =============================================================================
# USB / GPS Packet Parser
# =============================================================================

class USBPacketParser:
    """
    Parse GPS (and general) data arriving from the STM32 via USB CDC.

    Supports:
      - Text format: ``GPS:lat,lon,alt,pitch\\r\\n``
      - Binary format: ``GPSB`` header, 30 bytes total
    """

    def __init__(self):
        if CRCMOD_AVAILABLE:
            self.crc16_func = crcmod.mkCrcFun(
                0x11021, rev=False, initCrc=0xFFFF, xorOut=0x0000
            )
        else:
            self.crc16_func = None

    def parse_gps_data(self, data: bytes) -> Optional[GPSData]:
        """Attempt to parse GPS data from a raw USB CDC frame."""
        if not data:
            return None

        try:
            # Text format: "GPS:lat,lon,alt,pitch\r\n"
            text = data.decode("utf-8", errors="ignore").strip()
            if text.startswith("GPS:"):
                parts = text.split(":")[1].split(",")
                if len(parts) >= 4:
                    return GPSData(
                        latitude=float(parts[0]),
                        longitude=float(parts[1]),
                        altitude=float(parts[2]),
                        pitch=float(parts[3]),
                        timestamp=time.time(),
                    )

            # Binary format: [GPSB 4][lat 8][lon 8][alt 4][pitch 4][CRC 2] = 30 bytes
            if len(data) >= 30 and data[0:4] == b"GPSB":
                return self._parse_binary_gps(data)
        except Exception as e:
            logger.error(f"Error parsing GPS data: {e}")
        return None

    @staticmethod
    def _parse_binary_gps(data: bytes) -> Optional[GPSData]:
        """Parse 30-byte binary GPS frame."""
        try:
            if len(data) < 30:
                return None

            # Simple checksum CRC
            crc_rcv = (data[28] << 8) | data[29]
            crc_calc = sum(data[0:28]) & 0xFFFF
            if crc_rcv != crc_calc:
                logger.warning("GPS binary CRC mismatch")
                return None

            lat = struct.unpack(">d", data[4:12])[0]
            lon = struct.unpack(">d", data[12:20])[0]
            alt = struct.unpack(">f", data[20:24])[0]
            pitch = struct.unpack(">f", data[24:28])[0]

            return GPSData(
                latitude=lat,
                longitude=lon,
                altitude=alt,
                pitch=pitch,
                timestamp=time.time(),
            )
        except Exception as e:
            logger.error(f"Error parsing binary GPS: {e}")
            return None
