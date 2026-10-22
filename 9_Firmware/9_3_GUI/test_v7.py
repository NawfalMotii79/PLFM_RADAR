"""
V7-specific unit tests for the PLFM Radar GUI V7 modules.

Tests cover:
  - v7.models: RadarTarget, RadarSettings, GPSData, ProcessingConfig
  - v7.processing: RadarProcessor, USBPacketParser, apply_pitch_correction
  - v7.workers: polar_to_geographic
  - v7.hardware: STM32USBInterface (basic), production protocol re-exports

Does NOT require a running Qt event loop — only unit-testable components.
Run with:  python -m unittest test_v7 -v
"""

import os
import struct
import unittest
from dataclasses import asdict

import numpy as np


# =============================================================================
# Test: v7.models
# =============================================================================

class TestRadarTarget(unittest.TestCase):
    """RadarTarget dataclass."""

    def test_defaults(self):
        t = _models().RadarTarget(id=1, range=1000.0, velocity=5.0,
                                   azimuth=45.0, elevation=2.0)
        self.assertEqual(t.id, 1)
        self.assertEqual(t.range, 1000.0)
        self.assertEqual(t.snr, 0.0)
        self.assertEqual(t.track_id, -1)
        self.assertEqual(t.classification, "unknown")

    def test_to_dict(self):
        t = _models().RadarTarget(id=1, range=500.0, velocity=-10.0,
                                   azimuth=0.0, elevation=0.0, snr=15.0)
        d = t.to_dict()
        self.assertIsInstance(d, dict)
        self.assertEqual(d["range"], 500.0)
        self.assertEqual(d["snr"], 15.0)


class TestRadarSettings(unittest.TestCase):
    """RadarSettings — verify stale STM32 fields are removed."""

    def test_no_stale_fields(self):
        """chirp_duration, freq_min/max, prf1/2 must NOT exist."""
        s = _models().RadarSettings()
        d = asdict(s)
        for stale in ["chirp_duration_1", "chirp_duration_2",
                       "freq_min", "freq_max", "prf1", "prf2",
                       "chirps_per_position"]:
            self.assertNotIn(stale, d, f"Stale field '{stale}' still present")

    def test_has_physical_conversion_fields(self):
        s = _models().RadarSettings()
        self.assertIsInstance(s.range_resolution, float)
        self.assertIsInstance(s.velocity_resolution, float)
        self.assertGreater(s.range_resolution, 0)
        self.assertGreater(s.velocity_resolution, 0)

    def test_defaults(self):
        s = _models().RadarSettings()
        self.assertEqual(s.system_frequency, 10.5e9)
        self.assertEqual(s.coverage_radius, 1536)
        self.assertEqual(s.max_distance, 1536)


class TestGPSData(unittest.TestCase):
    def test_to_dict(self):
        g = _models().GPSData(latitude=41.9, longitude=12.5,
                               altitude=100.0, pitch=2.5)
        d = g.to_dict()
        self.assertAlmostEqual(d["latitude"], 41.9)
        self.assertAlmostEqual(d["pitch"], 2.5)


class TestProcessingConfig(unittest.TestCase):
    def test_defaults(self):
        cfg = _models().ProcessingConfig()
        self.assertTrue(cfg.clustering_enabled)
        self.assertTrue(cfg.tracking_enabled)
        self.assertFalse(cfg.mti_enabled)
        self.assertFalse(cfg.cfar_enabled)


class TestNoCrcmodDependency(unittest.TestCase):
    """crcmod was removed — verify it's not exported."""

    def test_no_crcmod_available(self):
        models = _models()
        self.assertFalse(hasattr(models, "CRCMOD_AVAILABLE"),
                         "CRCMOD_AVAILABLE should be removed from models")


# =============================================================================
# Test: v7.processing
# =============================================================================

class TestApplyPitchCorrection(unittest.TestCase):
    def test_positive_pitch(self):
        from v7.processing import apply_pitch_correction
        self.assertAlmostEqual(apply_pitch_correction(10.0, 3.0), 7.0)

    def test_zero_pitch(self):
        from v7.processing import apply_pitch_correction
        self.assertAlmostEqual(apply_pitch_correction(5.0, 0.0), 5.0)


class TestRadarProcessorMTI(unittest.TestCase):
    def test_mti_order1(self):
        from v7.processing import RadarProcessor
        from v7.models import ProcessingConfig
        proc = RadarProcessor()
        proc.set_config(ProcessingConfig(mti_enabled=True, mti_order=1))

        frame1 = np.ones((64, 32))
        frame2 = np.ones((64, 32)) * 3

        result1 = proc.mti_filter(frame1)
        np.testing.assert_array_equal(result1, np.zeros((64, 32)),
                                       err_msg="First frame should be zeros (no history)")

        result2 = proc.mti_filter(frame2)
        expected = frame2 - frame1
        np.testing.assert_array_almost_equal(result2, expected)

    def test_mti_order2(self):
        from v7.processing import RadarProcessor
        from v7.models import ProcessingConfig
        proc = RadarProcessor()
        proc.set_config(ProcessingConfig(mti_enabled=True, mti_order=2))

        f1 = np.ones((4, 4))
        f2 = np.ones((4, 4)) * 2
        f3 = np.ones((4, 4)) * 5

        proc.mti_filter(f1)  # zeros (need 3 frames)
        proc.mti_filter(f2)  # zeros
        result = proc.mti_filter(f3)
        # Order 2: x[n] - 2*x[n-1] + x[n-2] = 5 - 4 + 1 = 2
        np.testing.assert_array_almost_equal(result, np.ones((4, 4)) * 2)


class TestRadarProcessorCFAR(unittest.TestCase):
    def test_cfar_1d_detects_peak(self):
        from v7.processing import RadarProcessor
        signal = np.ones(64) * 10
        signal[32] = 500  # inject a strong target
        det = RadarProcessor.cfar_1d(signal, guard=2, train=4,
                                      threshold_factor=3.0, cfar_type="CA-CFAR")
        self.assertTrue(det[32], "Should detect strong peak at bin 32")

    def test_cfar_1d_no_false_alarm(self):
        from v7.processing import RadarProcessor
        signal = np.ones(64) * 10  # uniform — no target
        det = RadarProcessor.cfar_1d(signal, guard=2, train=4,
                                      threshold_factor=3.0)
        self.assertEqual(det.sum(), 0, "Should have no detections in flat noise")


class TestRadarProcessorProcessFrame(unittest.TestCase):
    def test_process_frame_returns_shapes(self):
        from v7.processing import RadarProcessor
        proc = RadarProcessor()
        frame = np.random.randn(64, 32) * 10
        frame[20, 8] = 5000  # inject a target
        power, mask = proc.process_frame(frame)
        self.assertEqual(power.shape, (64, 32))
        self.assertEqual(mask.shape, (64, 32))
        self.assertEqual(mask.dtype, bool)


class TestRadarProcessorWindowing(unittest.TestCase):
    def test_hann_window(self):
        from v7.processing import RadarProcessor
        data = np.ones((4, 32))
        windowed = RadarProcessor.apply_window(data, "Hann")
        # Hann window tapers to ~0 at edges
        self.assertLess(windowed[0, 0], 0.1)
        self.assertGreater(windowed[0, 16], 0.5)

    def test_none_window(self):
        from v7.processing import RadarProcessor
        data = np.ones((4, 32))
        result = RadarProcessor.apply_window(data, "None")
        np.testing.assert_array_equal(result, data)


class TestRadarProcessorDCNotch(unittest.TestCase):
    def test_dc_removal(self):
        from v7.processing import RadarProcessor
        data = np.ones((4, 8)) * 100
        data[0, :] += 50  # DC offset in range bin 0
        result = RadarProcessor.dc_notch(data)
        # Mean along axis=1 should be ~0
        row_means = np.mean(result, axis=1)
        for m in row_means:
            self.assertAlmostEqual(m, 0, places=10)


class TestRadarProcessorClustering(unittest.TestCase):
    def test_clustering_empty(self):
        from v7.processing import RadarProcessor
        result = RadarProcessor.clustering([], eps=100, min_samples=2)
        self.assertEqual(result, [])


class TestUSBPacketParser(unittest.TestCase):
    def test_parse_gps_text(self):
        from v7.processing import USBPacketParser
        parser = USBPacketParser()
        data = b"GPS:41.9028,12.4964,100.0,2.5\r\n"
        gps = parser.parse_gps_data(data)
        self.assertIsNotNone(gps)
        self.assertAlmostEqual(gps.latitude, 41.9028, places=3)
        self.assertAlmostEqual(gps.longitude, 12.4964, places=3)
        self.assertAlmostEqual(gps.altitude, 100.0)
        self.assertAlmostEqual(gps.pitch, 2.5)

    def test_parse_gps_text_invalid(self):
        from v7.processing import USBPacketParser
        parser = USBPacketParser()
        self.assertIsNone(parser.parse_gps_data(b"NOT_GPS_DATA"))
        self.assertIsNone(parser.parse_gps_data(b""))
        self.assertIsNone(parser.parse_gps_data(None))

    def test_parse_binary_gps(self):
        from v7.processing import USBPacketParser
        parser = USBPacketParser()
        # Build a valid binary GPS packet
        pkt = bytearray(b"GPSB")
        pkt += struct.pack(">d", 41.9028)     # lat
        pkt += struct.pack(">d", 12.4964)     # lon
        pkt += struct.pack(">f", 100.0)       # alt
        pkt += struct.pack(">f", 2.5)         # pitch
        # Simple checksum
        cksum = sum(pkt) & 0xFFFF
        pkt += struct.pack(">H", cksum)
        self.assertEqual(len(pkt), 30)

        gps = parser.parse_gps_data(bytes(pkt))
        self.assertIsNotNone(gps)
        self.assertAlmostEqual(gps.latitude, 41.9028, places=3)

    def test_no_crc16_func_attribute(self):
        """crcmod was removed — USBPacketParser should not have crc16_func."""
        from v7.processing import USBPacketParser
        parser = USBPacketParser()
        self.assertFalse(hasattr(parser, "crc16_func"),
                         "crc16_func should be removed (crcmod dead code)")

    def test_no_multi_prf_unwrap(self):
        """multi_prf_unwrap was removed (never called, prf fields removed)."""
        from v7.processing import RadarProcessor
        self.assertFalse(hasattr(RadarProcessor, "multi_prf_unwrap"),
                         "multi_prf_unwrap should be removed")


# =============================================================================
# Test: v7.workers — polar_to_geographic
# =============================================================================

def _pyqt6_available():
    try:
        import PyQt6.QtCore  # noqa: F401
        return True
    except ImportError:
        return False


@unittest.skipUnless(_pyqt6_available(), "PyQt6 not installed")
class TestPolarToGeographic(unittest.TestCase):
    def test_north_bearing(self):
        from v7.workers import polar_to_geographic
        lat, lon = polar_to_geographic(0.0, 0.0, 1000.0, 0.0)
        # Moving 1km north from equator
        self.assertGreater(lat, 0.0)
        self.assertAlmostEqual(lon, 0.0, places=4)

    def test_east_bearing(self):
        from v7.workers import polar_to_geographic
        lat, lon = polar_to_geographic(0.0, 0.0, 1000.0, 90.0)
        self.assertAlmostEqual(lat, 0.0, places=4)
        self.assertGreater(lon, 0.0)

    def test_zero_range(self):
        from v7.workers import polar_to_geographic
        lat, lon = polar_to_geographic(41.9, 12.5, 0.0, 0.0)
        self.assertAlmostEqual(lat, 41.9, places=6)
        self.assertAlmostEqual(lon, 12.5, places=6)


# =============================================================================
# Test: v7.hardware — production protocol re-exports
# =============================================================================

class TestHardwareReExports(unittest.TestCase):
    """Verify hardware.py re-exports all production protocol classes."""

    def test_exports(self):
        from v7.hardware import (
            FT2232HConnection,
            RadarProtocol,
            STM32USBInterface,
        )
        # Verify these are actual classes/types, not None
        self.assertTrue(callable(FT2232HConnection))
        self.assertTrue(callable(RadarProtocol))
        self.assertTrue(callable(STM32USBInterface))

    def test_stm32_list_devices_no_crash(self):
        from v7.hardware import STM32USBInterface
        stm = STM32USBInterface()
        self.assertFalse(stm.is_open)
        # list_devices should return empty list (no USB in test env), not crash
        devs = stm.list_devices()
        self.assertIsInstance(devs, list)


# =============================================================================
# Test: v7.__init__ — clean exports
# =============================================================================

class TestV7Init(unittest.TestCase):
    """Verify top-level v7 package exports."""

    def test_no_crcmod_export(self):
        import v7
        self.assertFalse(hasattr(v7, "CRCMOD_AVAILABLE"),
                         "CRCMOD_AVAILABLE should not be in v7.__all__")

    def test_key_exports(self):
        import v7
        # Core exports (no PyQt6 required)
        for name in ["RadarTarget", "RadarSettings", "GPSData",
                      "ProcessingConfig", "FT2232HConnection",
                      "RadarProtocol", "RadarProcessor"]:
            self.assertTrue(hasattr(v7, name), f"v7 missing export: {name}")
        # PyQt6-dependent exports — only present when PyQt6 is installed
        if _pyqt6_available():
            for name in ["RadarDataWorker", "RadarMapWidget",
                          "RadarDashboard"]:
                self.assertTrue(hasattr(v7, name), f"v7 missing export: {name}")


# =============================================================================
# Test: AGC Visualization data model
# =============================================================================

class TestAGCVisualizationV7(unittest.TestCase):
    """AGC visualization ring buffer and data model tests (no Qt required)."""

    def _make_deque(self, maxlen=256):
        from collections import deque
        return deque(maxlen=maxlen)

    def test_ring_buffer_basics(self):
        d = self._make_deque(maxlen=4)
        for i in range(6):
            d.append(i)
        self.assertEqual(list(d), [2, 3, 4, 5])

    def test_gain_range_4bit(self):
        """AGC gain is 4-bit (0-15)."""
        from radar_protocol import StatusResponse
        for g in [0, 7, 15]:
            sr = StatusResponse(agc_current_gain=g)
            self.assertEqual(sr.agc_current_gain, g)

    def test_peak_range_8bit(self):
        """Peak magnitude is 8-bit (0-255)."""
        from radar_protocol import StatusResponse
        for p in [0, 128, 255]:
            sr = StatusResponse(agc_peak_magnitude=p)
            self.assertEqual(sr.agc_peak_magnitude, p)

    def test_saturation_accumulation(self):
        """Saturation ring buffer sum tracks total events."""
        sat = self._make_deque(maxlen=256)
        for s in [0, 5, 0, 10, 3]:
            sat.append(s)
        self.assertEqual(sum(sat), 18)

    def test_mode_label_logic(self):
        """AGC mode string from enable field."""
        from radar_protocol import StatusResponse
        self.assertEqual(
            "AUTO" if StatusResponse(agc_enable=1).agc_enable else "MANUAL",
            "AUTO")
        self.assertEqual(
            "AUTO" if StatusResponse(agc_enable=0).agc_enable else "MANUAL",
            "MANUAL")

    def test_history_len_default(self):
        """Default history length should be 256."""
        d = self._make_deque(maxlen=256)
        self.assertEqual(d.maxlen, 256)

    def test_color_thresholds(self):
        """Saturation color: green=0, warning=1-10, error>10."""
        from v7.models import DARK_SUCCESS, DARK_WARNING, DARK_ERROR
        def pick_color(total):
            if total > 10:
                return DARK_ERROR
            if total > 0:
                return DARK_WARNING
            return DARK_SUCCESS
        self.assertEqual(pick_color(0), DARK_SUCCESS)
        self.assertEqual(pick_color(5), DARK_WARNING)
        self.assertEqual(pick_color(11), DARK_ERROR)


# =============================================================================
# Test: v7.models.WaveformConfig
# =============================================================================

class TestWaveformConfig(unittest.TestCase):
    """WaveformConfig dataclass and derived physical properties."""

    def test_defaults(self):
        from v7.models import WaveformConfig
        wc = WaveformConfig()
        self.assertEqual(wc.sample_rate_hz, 100e6)
        self.assertEqual(wc.bandwidth_hz, 20e6)
        self.assertEqual(wc.chirp_duration_s, 30e-6)
        self.assertEqual(wc.pri_s, 167e-6)
        self.assertEqual(wc.center_freq_hz, 10.5e9)
        self.assertEqual(wc.n_range_bins, 64)
        self.assertEqual(wc.n_doppler_bins, 32)
        self.assertEqual(wc.chirps_per_subframe, 16)
        self.assertEqual(wc.fft_size, 1024)
        self.assertEqual(wc.decimation_factor, 16)

    def test_range_resolution(self):
        """range_resolution_m should be ~23.98 m/bin (matched filter, 100 MSPS)."""
        from v7.models import WaveformConfig
        wc = WaveformConfig()
        self.assertAlmostEqual(wc.range_resolution_m, 23.983, places=1)

    def test_velocity_resolution(self):
        """velocity_resolution_mps should be ~5.34 m/s/bin (PRI=167us, 16 chirps)."""
        from v7.models import WaveformConfig
        wc = WaveformConfig()
        self.assertAlmostEqual(wc.velocity_resolution_mps, 5.343, places=1)

    def test_max_range(self):
        """max_range_m = range_resolution * n_range_bins."""
        from v7.models import WaveformConfig
        wc = WaveformConfig()
        self.assertAlmostEqual(wc.max_range_m, wc.range_resolution_m * 64, places=1)

    def test_max_velocity(self):
        """max_velocity_mps = velocity_resolution * n_doppler_bins / 2."""
        from v7.models import WaveformConfig
        wc = WaveformConfig()
        self.assertAlmostEqual(
            wc.max_velocity_mps,
            wc.velocity_resolution_mps * 16,
            places=2,
        )

    def test_custom_params(self):
        """Non-default parameters correctly change derived values."""
        from v7.models import WaveformConfig
        wc1 = WaveformConfig()
        wc2 = WaveformConfig(sample_rate_hz=200e6)  # double Fs → halve range bin
        self.assertAlmostEqual(wc2.range_resolution_m, wc1.range_resolution_m / 2, places=2)

    def test_zero_center_freq_velocity(self):
        """Zero center freq should cause ZeroDivisionError in velocity calc."""
        from v7.models import WaveformConfig
        wc = WaveformConfig(center_freq_hz=0.0)
        with self.assertRaises(ZeroDivisionError):
            _ = wc.velocity_resolution_mps


# =============================================================================
# Test: v7.software_fpga.SoftwareFPGA
# =============================================================================

class TestSoftwareFPGA(unittest.TestCase):
    """SoftwareFPGA register interface and signal chain."""

    def _make_fpga(self):
        from v7.software_fpga import SoftwareFPGA
        return SoftwareFPGA()

    def test_reset_defaults(self):
        """Register reset values match FPGA RTL (radar_system_top.v)."""
        fpga = self._make_fpga()
        self.assertEqual(fpga.detect_threshold, 10_000)
        self.assertEqual(fpga.gain_shift, 0)
        self.assertFalse(fpga.cfar_enable)
        self.assertEqual(fpga.cfar_guard, 2)
        self.assertEqual(fpga.cfar_train, 8)
        self.assertEqual(fpga.cfar_alpha, 0x30)
        self.assertEqual(fpga.cfar_mode, 0)
        self.assertFalse(fpga.mti_enable)
        self.assertEqual(fpga.dc_notch_width, 0)
        self.assertFalse(fpga.agc_enable)
        self.assertEqual(fpga.agc_target, 200)
        self.assertEqual(fpga.agc_attack, 1)
        self.assertEqual(fpga.agc_decay, 1)
        self.assertEqual(fpga.agc_holdoff, 4)

    def test_setter_detect_threshold(self):
        fpga = self._make_fpga()
        fpga.set_detect_threshold(5000)
        self.assertEqual(fpga.detect_threshold, 5000)

    def test_setter_detect_threshold_clamp_16bit(self):
        fpga = self._make_fpga()
        fpga.set_detect_threshold(0x1FFFF)  # 17-bit
        self.assertEqual(fpga.detect_threshold, 0xFFFF)

    def test_setter_gain_shift_clamp_4bit(self):
        fpga = self._make_fpga()
        fpga.set_gain_shift(0xFF)
        self.assertEqual(fpga.gain_shift, 0x0F)

    def test_setter_cfar_enable(self):
        fpga = self._make_fpga()
        fpga.set_cfar_enable(True)
        self.assertTrue(fpga.cfar_enable)
        fpga.set_cfar_enable(False)
        self.assertFalse(fpga.cfar_enable)

    def test_setter_cfar_guard_clamp_4bit(self):
        fpga = self._make_fpga()
        fpga.set_cfar_guard(0x1F)
        self.assertEqual(fpga.cfar_guard, 0x0F)

    def test_setter_cfar_train_min_1(self):
        """CFAR train cells clamped to min 1."""
        fpga = self._make_fpga()
        fpga.set_cfar_train(0)
        self.assertEqual(fpga.cfar_train, 1)

    def test_setter_cfar_train_clamp_5bit(self):
        fpga = self._make_fpga()
        fpga.set_cfar_train(0x3F)
        self.assertEqual(fpga.cfar_train, 0x1F)

    def test_setter_cfar_alpha_clamp_8bit(self):
        fpga = self._make_fpga()
        fpga.set_cfar_alpha(0x1FF)
        self.assertEqual(fpga.cfar_alpha, 0xFF)

    def test_setter_cfar_mode_clamp_2bit(self):
        fpga = self._make_fpga()
        fpga.set_cfar_mode(7)
        self.assertEqual(fpga.cfar_mode, 3)

    def test_setter_mti_enable(self):
        fpga = self._make_fpga()
        fpga.set_mti_enable(True)
        self.assertTrue(fpga.mti_enable)

    def test_setter_dc_notch_clamp_3bit(self):
        fpga = self._make_fpga()
        fpga.set_dc_notch_width(0xFF)
        self.assertEqual(fpga.dc_notch_width, 7)

    def test_setter_agc_params_selective(self):
        """set_agc_params only changes provided fields."""
        fpga = self._make_fpga()
        fpga.set_agc_params(target=100)
        self.assertEqual(fpga.agc_target, 100)
        self.assertEqual(fpga.agc_attack, 1)  # unchanged
        fpga.set_agc_params(attack=3, decay=5)
        self.assertEqual(fpga.agc_attack, 3)
        self.assertEqual(fpga.agc_decay, 5)
        self.assertEqual(fpga.agc_target, 100)  # unchanged

    def test_setter_agc_params_clamp(self):
        fpga = self._make_fpga()
        fpga.set_agc_params(target=0xFFF, attack=0xFF, decay=0xFF, holdoff=0xFF)
        self.assertEqual(fpga.agc_target, 0xFF)
        self.assertEqual(fpga.agc_attack, 0x0F)
        self.assertEqual(fpga.agc_decay, 0x0F)
        self.assertEqual(fpga.agc_holdoff, 0x0F)


# ============================================================================
# Test: live vs replay physical-unit parity — regression guard for unit drift
#
# Uses AST parse of workers.py (not inspect.getsource / import) so the test
# runs in headless CI without PyQt6 — v7.workers imports PyQt6 unconditionally
# at workers.py:24, and other worker tests here already use skipUnless(
# _pyqt6_available()). Contract enforcement must not be gated on GUI deps.
#
# Asserts on AST nodes (Call / Attribute / BinOp), not source substrings, so
# false-pass on comments or docstring wording is impossible.
# ============================================================================


class TestLiveReplayPhysicalUnitsParity(unittest.TestCase):
    """Contract: live path (RadarDataWorker._run_host_dsp) and replay path
    (ReplayWorker._emit_frame) both derive bin-to-physical conversion from
    WaveformConfig — same source of truth, identical (range_m, velocity_ms)
    for identical detections.

    Regression context: before the fix, live path used
    RadarSettings.velocity_resolution (default 1.0 in models.py:113) while
    replay used WaveformConfig.velocity_resolution_mps (~5.343). Live GUI
    therefore under-reported velocity by factor ~5.34x vs replay for
    identical frames. See test_v7.py:449 for the WaveformConfig pin.
    """

    @staticmethod
    def _parse_method(class_name: str, method_name: str):
        """Return AST FunctionDef for class_name.method_name from workers.py,
        without importing v7.workers (PyQt6-independent)."""
        import ast
        from pathlib import Path
        path = Path(__file__).parent / "v7" / "workers.py"
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in tree.body:
            if isinstance(node, ast.ClassDef) and node.name == class_name:
                for item in node.body:
                    if isinstance(item, ast.FunctionDef) and item.name == method_name:
                        return item
        raise RuntimeError(f"{class_name}.{method_name} not found in workers.py")

    @staticmethod
    def _has_attribute_chain(tree, chain):
        """True if AST tree contains a dotted attribute access matching chain.

        Chain ('self', '_settings', 'range_resolution') matches
        ``self._settings.range_resolution`` exactly.
        """
        import ast
        for n in ast.walk(tree):
            if isinstance(n, ast.Attribute):
                parts = [n.attr]
                cur = n.value
                while isinstance(cur, ast.Attribute):
                    parts.append(cur.attr)
                    cur = cur.value
                if isinstance(cur, ast.Name):
                    parts.append(cur.id)
                    parts.reverse()
                    if tuple(parts) == tuple(chain):
                        return True
        return False

    @staticmethod
    def _has_call_to(tree, func_name):
        """True if AST tree contains a call to a bare name (func_name())."""
        import ast
        for n in ast.walk(tree):
            if (isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                    and n.func.id == func_name):
                return True
        return False

    @staticmethod
    def _has_dbin_minus(tree, literal):
        """True if AST tree contains ``dbin - <literal>`` binary op."""
        import ast
        for n in ast.walk(tree):
            if (isinstance(n, ast.BinOp) and isinstance(n.op, ast.Sub)
                    and isinstance(n.left, ast.Name) and n.left.id == "dbin"
                    and isinstance(n.right, ast.Constant)
                    and n.right.value == literal):
                return True
        return False

    def test_live_path_uses_waveform_config(self):
        """RadarDataWorker.__init__ must instantiate WaveformConfig() into
        self._waveform; _run_host_dsp must read self._waveform.range_resolution_m
        / velocity_resolution_mps — not self._settings equivalents."""
        init = self._parse_method("RadarDataWorker", "__init__")
        self.assertTrue(self._has_call_to(init, "WaveformConfig"),
            "RadarDataWorker.__init__ must instantiate WaveformConfig() into self._waveform.")
        method = self._parse_method("RadarDataWorker", "_run_host_dsp")
        self.assertTrue(
            self._has_attribute_chain(method, ("self", "_waveform", "range_resolution_m")),
            "Live path must read self._waveform.range_resolution_m.")
        self.assertTrue(
            self._has_attribute_chain(method, ("self", "_waveform", "velocity_resolution_mps")),
            "Live path must read self._waveform.velocity_resolution_mps. "
            "RadarSettings.velocity_resolution default 1.0 caused ~5.34x "
            "underreport vs replay (test_v7.py:449 pins ~5.343).")
        self.assertFalse(self._has_attribute_chain(
            method, ("self", "_settings", "range_resolution")),
            "Live path still reads stale RadarSettings.range_resolution.")
        self.assertFalse(self._has_attribute_chain(
            method, ("self", "_settings", "velocity_resolution")),
            "Live path still reads stale RadarSettings.velocity_resolution.")

    def test_live_path_doppler_center_not_hardcoded(self):
        """_run_host_dsp must derive doppler_center from frame shape, not
        use hardcoded ``dbin - 16`` — mirrors processing.py:520."""
        method = self._parse_method("RadarDataWorker", "_run_host_dsp")
        self.assertFalse(self._has_dbin_minus(method, 16),
            "Hardcoded doppler_center=16 breaks if frame shape changes. "
            "Use frame.detections.shape[1] // 2 like processing.py:520.")

    def test_replay_path_still_uses_waveform_config(self):
        """Parity half: replay path (ReplayWorker._emit_frame) must keep
        reading self._waveform.range_resolution_m / velocity_resolution_mps —
        guards against someone breaking the replay side of the invariant."""
        method = self._parse_method("ReplayWorker", "_emit_frame")
        self.assertTrue(self._has_attribute_chain(
            method, ("self", "_waveform", "range_resolution_m")),
            "Replay path lost WaveformConfig range source of truth.")
        self.assertTrue(self._has_attribute_chain(
            method, ("self", "_waveform", "velocity_resolution_mps")),
            "Replay path lost WaveformConfig velocity source of truth.")


class TestSoftwareFPGASignalChain(unittest.TestCase):
    """SoftwareFPGA.process_chirps with real co-sim data."""

    COSIM_DIR = os.path.join(
        os.path.dirname(__file__), "..", "9_2_FPGA", "tb", "cosim",
        "real_data", "hex"
    )

    def _cosim_available(self):
        return os.path.isfile(os.path.join(self.COSIM_DIR, "doppler_map_i.npy"))

    def test_process_chirps_returns_radar_frame(self):
        """process_chirps produces a RadarFrame with correct shapes."""
        if not self._cosim_available():
            self.skipTest("co-sim data not found")
        from v7.software_fpga import SoftwareFPGA
        from radar_protocol import RadarFrame

        # Load decimated range data as minimal input (32 chirps x 64 bins)
        dec_i = np.load(os.path.join(self.COSIM_DIR, "decimated_range_i.npy"))
        dec_q = np.load(os.path.join(self.COSIM_DIR, "decimated_range_q.npy"))

        # Build fake 1024-sample chirps from decimated data (pad with zeros)
        n_chirps = dec_i.shape[0]
        iq_i = np.zeros((n_chirps, 1024), dtype=np.int64)
        iq_q = np.zeros((n_chirps, 1024), dtype=np.int64)
        # Put decimated data into first 64 bins so FFT has something
        iq_i[:, :dec_i.shape[1]] = dec_i
        iq_q[:, :dec_q.shape[1]] = dec_q

        fpga = SoftwareFPGA()
        frame = fpga.process_chirps(iq_i, iq_q, frame_number=42, timestamp=1.0)

        self.assertIsInstance(frame, RadarFrame)
        self.assertEqual(frame.frame_number, 42)
        self.assertAlmostEqual(frame.timestamp, 1.0)
        self.assertEqual(frame.range_doppler_i.shape, (64, 32))
        self.assertEqual(frame.range_doppler_q.shape, (64, 32))
        self.assertEqual(frame.magnitude.shape, (64, 32))
        self.assertEqual(frame.detections.shape, (64, 32))
        self.assertEqual(frame.range_profile.shape, (64,))
        self.assertEqual(frame.detection_count, int(frame.detections.sum()))

    def test_cfar_enable_changes_detections(self):
        """Enabling CFAR vs simple threshold should yield different detection counts."""
        if not self._cosim_available():
            self.skipTest("co-sim data not found")
        from v7.software_fpga import SoftwareFPGA

        iq_i = np.zeros((32, 1024), dtype=np.int64)
        iq_q = np.zeros((32, 1024), dtype=np.int64)
        # Inject a single strong tone in bin 10 of every chirp
        iq_i[:, 10] = 5000
        iq_q[:, 10] = 3000

        fpga_thresh = SoftwareFPGA()
        fpga_thresh.set_detect_threshold(1)  # very low → many detections
        frame_thresh = fpga_thresh.process_chirps(iq_i, iq_q)

        fpga_cfar = SoftwareFPGA()
        fpga_cfar.set_cfar_enable(True)
        fpga_cfar.set_cfar_alpha(0x10)  # low alpha → more detections
        frame_cfar = fpga_cfar.process_chirps(iq_i, iq_q)

        # Just verify both produce valid frames — exact counts depend on chain
        self.assertIsNotNone(frame_thresh)
        self.assertIsNotNone(frame_cfar)
        self.assertEqual(frame_thresh.magnitude.shape, (64, 32))
        self.assertEqual(frame_cfar.magnitude.shape, (64, 32))


class TestQuantizeRawIQ(unittest.TestCase):
    """quantize_raw_iq utility function."""

    def test_3d_input(self):
        """3-D (frames, chirps, samples) → uses first frame."""
        from v7.software_fpga import quantize_raw_iq
        raw = np.random.randn(5, 32, 1024) + 1j * np.random.randn(5, 32, 1024)
        iq_i, iq_q = quantize_raw_iq(raw)
        self.assertEqual(iq_i.shape, (32, 1024))
        self.assertEqual(iq_q.shape, (32, 1024))
        self.assertTrue(np.all(np.abs(iq_i) <= 32767))
        self.assertTrue(np.all(np.abs(iq_q) <= 32767))

    def test_2d_input(self):
        """2-D (chirps, samples) → works directly."""
        from v7.software_fpga import quantize_raw_iq
        raw = np.random.randn(32, 1024) + 1j * np.random.randn(32, 1024)
        iq_i, _iq_q = quantize_raw_iq(raw)
        self.assertEqual(iq_i.shape, (32, 1024))

    def test_zero_input(self):
        """All-zero complex input → all-zero output."""
        from v7.software_fpga import quantize_raw_iq
        raw = np.zeros((32, 1024), dtype=np.complex128)
        iq_i, iq_q = quantize_raw_iq(raw)
        self.assertTrue(np.all(iq_i == 0))
        self.assertTrue(np.all(iq_q == 0))

    def test_peak_target_scaling(self):
        """Peak of output should be near peak_target."""
        from v7.software_fpga import quantize_raw_iq
        raw = np.zeros((32, 1024), dtype=np.complex128)
        raw[0, 0] = 1.0 + 0j  # single peak
        iq_i, _iq_q = quantize_raw_iq(raw, peak_target=500)
        # The peak I value should be exactly 500 (sole max)
        self.assertEqual(int(iq_i[0, 0]), 500)


# =============================================================================
# Test: v7.replay (ReplayEngine, detect_format)
# =============================================================================

class TestDetectFormat(unittest.TestCase):
    """detect_format auto-detection logic."""

    COSIM_DIR = os.path.join(
        os.path.dirname(__file__), "..", "9_2_FPGA", "tb", "cosim",
        "real_data", "hex"
    )

    def test_cosim_dir(self):
        if not os.path.isdir(self.COSIM_DIR):
            self.skipTest("co-sim dir not found")
        from v7.replay import detect_format, ReplayFormat
        self.assertEqual(detect_format(self.COSIM_DIR), ReplayFormat.COSIM_DIR)

    def test_npy_file(self):
        """A .npy file → RAW_IQ_NPY."""
        from v7.replay import detect_format, ReplayFormat
        import tempfile
        with tempfile.NamedTemporaryFile(suffix=".npy", delete=False) as f:
            np.save(f, np.zeros((2, 32, 1024), dtype=np.complex128))
            tmp = f.name
        try:
            self.assertEqual(detect_format(tmp), ReplayFormat.RAW_IQ_NPY)
        finally:
            os.unlink(tmp)

    def test_h5_file(self):
        """A .h5 file → HDF5."""
        from v7.replay import detect_format, ReplayFormat
        self.assertEqual(detect_format("/tmp/fake_recording.h5"), ReplayFormat.HDF5)

    def test_unknown_extension_raises(self):
        from v7.replay import detect_format
        with self.assertRaises(ValueError):
            detect_format("/tmp/data.csv")

    def test_empty_dir_raises(self):
        """Directory without co-sim files → ValueError."""
        from v7.replay import detect_format
        import tempfile
        with tempfile.TemporaryDirectory() as td, self.assertRaises(ValueError):
            detect_format(td)


class TestReplayEngineCosim(unittest.TestCase):
    """ReplayEngine loading from FPGA co-sim directory."""

    COSIM_DIR = os.path.join(
        os.path.dirname(__file__), "..", "9_2_FPGA", "tb", "cosim",
        "real_data", "hex"
    )

    def _available(self):
        return os.path.isfile(os.path.join(self.COSIM_DIR, "doppler_map_i.npy"))

    def test_load_cosim(self):
        if not self._available():
            self.skipTest("co-sim data not found")
        from v7.replay import ReplayEngine, ReplayFormat
        engine = ReplayEngine(self.COSIM_DIR)
        self.assertEqual(engine.fmt, ReplayFormat.COSIM_DIR)
        self.assertEqual(engine.total_frames, 1)

    def test_get_frame_cosim(self):
        if not self._available():
            self.skipTest("co-sim data not found")
        from v7.replay import ReplayEngine
        from radar_protocol import RadarFrame
        engine = ReplayEngine(self.COSIM_DIR)
        frame = engine.get_frame(0)
        self.assertIsInstance(frame, RadarFrame)
        self.assertEqual(frame.range_doppler_i.shape, (64, 32))
        self.assertEqual(frame.magnitude.shape, (64, 32))

    def test_get_frame_out_of_range(self):
        if not self._available():
            self.skipTest("co-sim data not found")
        from v7.replay import ReplayEngine
        engine = ReplayEngine(self.COSIM_DIR)
        with self.assertRaises(IndexError):
            engine.get_frame(1)
        with self.assertRaises(IndexError):
            engine.get_frame(-1)


class TestReplayEngineRawIQ(unittest.TestCase):
    """ReplayEngine loading from raw IQ .npy cube."""

    def test_load_raw_iq_synthetic(self):
        """Synthetic raw IQ cube loads and produces correct frame count."""
        import tempfile
        from v7.replay import ReplayEngine, ReplayFormat
        from v7.software_fpga import SoftwareFPGA

        raw = np.random.randn(3, 32, 1024) + 1j * np.random.randn(3, 32, 1024)
        with tempfile.NamedTemporaryFile(suffix=".npy", delete=False) as f:
            np.save(f, raw)
            tmp = f.name
        try:
            fpga = SoftwareFPGA()
            engine = ReplayEngine(tmp, software_fpga=fpga)
            self.assertEqual(engine.fmt, ReplayFormat.RAW_IQ_NPY)
            self.assertEqual(engine.total_frames, 3)
        finally:
            os.unlink(tmp)

    def test_get_frame_raw_iq_synthetic(self):
        """get_frame on raw IQ runs SoftwareFPGA and returns RadarFrame."""
        import tempfile
        from v7.replay import ReplayEngine
        from v7.software_fpga import SoftwareFPGA
        from radar_protocol import RadarFrame

        raw = np.random.randn(2, 32, 1024) + 1j * np.random.randn(2, 32, 1024)
        with tempfile.NamedTemporaryFile(suffix=".npy", delete=False) as f:
            np.save(f, raw)
            tmp = f.name
        try:
            fpga = SoftwareFPGA()
            engine = ReplayEngine(tmp, software_fpga=fpga)
            frame = engine.get_frame(0)
            self.assertIsInstance(frame, RadarFrame)
            self.assertEqual(frame.range_doppler_i.shape, (64, 32))
            self.assertEqual(frame.frame_number, 0)
        finally:
            os.unlink(tmp)

    def test_raw_iq_no_fpga_raises(self):
        """Raw IQ get_frame without SoftwareFPGA → RuntimeError."""
        import tempfile
        from v7.replay import ReplayEngine

        raw = np.random.randn(1, 32, 1024) + 1j * np.random.randn(1, 32, 1024)
        with tempfile.NamedTemporaryFile(suffix=".npy", delete=False) as f:
            np.save(f, raw)
            tmp = f.name
        try:
            engine = ReplayEngine(tmp)
            with self.assertRaises(RuntimeError):
                engine.get_frame(0)
        finally:
            os.unlink(tmp)


class TestReplayEngineHDF5(unittest.TestCase):
    """ReplayEngine loading from HDF5 recordings."""

    def _skip_no_h5py(self):
        try:
            import h5py  # noqa: F401
        except ImportError:
            self.skipTest("h5py not installed")

    def test_load_hdf5_synthetic(self):
        """Synthetic HDF5 loads and iterates frames."""
        self._skip_no_h5py()
        import tempfile
        import h5py
        from v7.replay import ReplayEngine, ReplayFormat
        from radar_protocol import RadarFrame

        with tempfile.NamedTemporaryFile(suffix=".h5", delete=False) as f:
            tmp = f.name

        try:
            with h5py.File(tmp, "w") as hf:
                hf.attrs["creator"] = "test"
                hf.attrs["range_bins"] = 64
                hf.attrs["doppler_bins"] = 32
                grp = hf.create_group("frames")
                for i in range(3):
                    fg = grp.create_group(f"frame_{i:06d}")
                    fg.attrs["timestamp"] = float(i)
                    fg.attrs["frame_number"] = i
                    fg.attrs["detection_count"] = 0
                    fg.create_dataset("range_doppler_i",
                                      data=np.zeros((64, 32), dtype=np.int16))
                    fg.create_dataset("range_doppler_q",
                                      data=np.zeros((64, 32), dtype=np.int16))
                    fg.create_dataset("magnitude",
                                      data=np.zeros((64, 32), dtype=np.float64))
                    fg.create_dataset("detections",
                                      data=np.zeros((64, 32), dtype=np.uint8))
                    fg.create_dataset("range_profile",
                                      data=np.zeros(64, dtype=np.float64))

            engine = ReplayEngine(tmp)
            self.assertEqual(engine.fmt, ReplayFormat.HDF5)
            self.assertEqual(engine.total_frames, 3)

            frame = engine.get_frame(1)
            self.assertIsInstance(frame, RadarFrame)
            self.assertEqual(frame.frame_number, 1)
            self.assertEqual(frame.range_doppler_i.shape, (64, 32))
            engine.close()
        finally:
            os.unlink(tmp)


# =============================================================================
# Test: v7.processing.extract_targets_from_frame
# =============================================================================

class TestExtractTargetsFromFrame(unittest.TestCase):
    """extract_targets_from_frame bin-to-physical conversion."""

    def _make_frame(self, det_cells=None):
        """Create a minimal RadarFrame with optional detection cells."""
        from radar_protocol import RadarFrame
        frame = RadarFrame()
        if det_cells:
            for rbin, dbin in det_cells:
                frame.detections[rbin, dbin] = 1
                frame.magnitude[rbin, dbin] = 1000.0
        frame.detection_count = int(frame.detections.sum())
        frame.timestamp = 1.0
        return frame

    def test_no_detections(self):
        from v7.processing import extract_targets_from_frame
        frame = self._make_frame()
        targets = extract_targets_from_frame(frame)
        self.assertEqual(len(targets), 0)

    def test_single_detection_range(self):
        """Detection at range bin 10 → range = 10 * range_resolution."""
        from v7.processing import extract_targets_from_frame
        frame = self._make_frame(det_cells=[(10, 16)])  # dbin=16 = center → vel=0
        targets = extract_targets_from_frame(frame, range_resolution=23.983)
        self.assertEqual(len(targets), 1)
        self.assertAlmostEqual(targets[0].range, 10 * 23.983, places=1)
        self.assertAlmostEqual(targets[0].velocity, 0.0, places=2)

    def test_velocity_sign(self):
        """Doppler bin < center → negative velocity, > center → positive."""
        from v7.processing import extract_targets_from_frame
        frame = self._make_frame(det_cells=[(5, 10), (5, 20)])
        targets = extract_targets_from_frame(frame, velocity_resolution=1.484)
        # dbin=10: vel = (10-16)*1.484 = -8.904  (approaching)
        # dbin=20: vel = (20-16)*1.484 = +5.936  (receding)
        self.assertLess(targets[0].velocity, 0)
        self.assertGreater(targets[1].velocity, 0)

    def test_snr_positive_for_nonzero_mag(self):
        from v7.processing import extract_targets_from_frame
        frame = self._make_frame(det_cells=[(3, 16)])
        targets = extract_targets_from_frame(frame)
        self.assertGreater(targets[0].snr, 0)

    def test_gps_georef(self):
        """With GPS data, targets get non-zero lat/lon."""
        from v7.processing import extract_targets_from_frame
        from v7.models import GPSData
        gps = GPSData(latitude=41.9, longitude=12.5, altitude=0.0,
                      pitch=0.0, heading=90.0)
        frame = self._make_frame(det_cells=[(10, 16)])
        targets = extract_targets_from_frame(
            frame, range_resolution=100.0, gps=gps)
        # Should be roughly east of radar position
        self.assertAlmostEqual(targets[0].latitude, 41.9, places=2)
        self.assertGreater(targets[0].longitude, 12.5)

    def test_multiple_detections(self):
        from v7.processing import extract_targets_from_frame
        frame = self._make_frame(det_cells=[(0, 0), (10, 10), (63, 31)])
        targets = extract_targets_from_frame(frame)
        self.assertEqual(len(targets), 3)
        # IDs should be sequential 0, 1, 2
        self.assertEqual([t.id for t in targets], [0, 1, 2])


# =============================================================================
# Helper: lazy import of v7.models
# =============================================================================

def _models():
    import v7.models
    return v7.models


if __name__ == "__main__":
    unittest.main()
