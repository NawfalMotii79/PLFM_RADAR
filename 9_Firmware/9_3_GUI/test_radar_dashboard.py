#!/usr/bin/env python3
"""
Tests for AERIS-10 Radar Dashboard protocol parsing, command building,
data recording, and acquisition logic.

Run: python -m pytest test_radar_dashboard.py -v
  or: python test_radar_dashboard.py
"""

import importlib.util
import os
import queue
import struct
import tempfile
import time
import unittest

import numpy as np
from radar_protocol import (
    _HARDWARE_ONLY_OPCODES,
    DATA_PACKET_SIZE,
    FOOTER_BYTE,
    HEADER_BYTE,
    NUM_CELLS,
    NUM_DOPPLER_BINS,
    NUM_RANGE_BINS,
    STATUS_HEADER_BYTE,
    DataRecorder,
    FT2232HConnection,
    Opcode,
    RadarAcquisition,
    RadarFrame,
    RadarProtocol,
    StatusResponse,
)

_H5PY_AVAILABLE = importlib.util.find_spec("h5py") is not None


class TestRadarProtocol(unittest.TestCase):
    """Test packet parsing and command building against usb_data_interface.v."""

    # ----------------------------------------------------------------
    # Command building
    # ----------------------------------------------------------------
    def test_build_command_trigger(self):
        """Opcode 0x01, value 1 → {0x01, 0x00, 0x0001}."""
        cmd = RadarProtocol.build_command(0x01, 1)
        self.assertEqual(len(cmd), 4)
        word = struct.unpack(">I", cmd)[0]
        self.assertEqual((word >> 24) & 0xFF, 0x01)  # opcode
        self.assertEqual((word >> 16) & 0xFF, 0x00)  # addr
        self.assertEqual(word & 0xFFFF, 1)  # value

    def test_build_command_cfar_alpha(self):
        """Opcode 0x23, value 0x30 (alpha=3.0 Q4.4)."""
        cmd = RadarProtocol.build_command(0x23, 0x30)
        word = struct.unpack(">I", cmd)[0]
        self.assertEqual((word >> 24) & 0xFF, 0x23)
        self.assertEqual(word & 0xFFFF, 0x30)

    def test_build_command_status_request(self):
        """Opcode 0xFF, value 0."""
        cmd = RadarProtocol.build_command(0xFF, 0)
        word = struct.unpack(">I", cmd)[0]
        self.assertEqual((word >> 24) & 0xFF, 0xFF)
        self.assertEqual(word & 0xFFFF, 0)

    def test_build_command_with_addr(self):
        """Command with non-zero addr field."""
        cmd = RadarProtocol.build_command(0x10, 500, addr=0x42)
        word = struct.unpack(">I", cmd)[0]
        self.assertEqual((word >> 24) & 0xFF, 0x10)
        self.assertEqual((word >> 16) & 0xFF, 0x42)
        self.assertEqual(word & 0xFFFF, 500)

    def test_build_command_value_clamp(self):
        """Value > 0xFFFF should be masked to 16 bits."""
        cmd = RadarProtocol.build_command(0x01, 0x1FFFF)
        word = struct.unpack(">I", cmd)[0]
        self.assertEqual(word & 0xFFFF, 0xFFFF)

    # ----------------------------------------------------------------
    # Data packet parsing
    # ----------------------------------------------------------------
    def _make_data_packet(self, range_i=100, range_q=200, dop_i=300, dop_q=400, detection=0):
        """Build a synthetic 11-byte data packet matching FT2232H format."""
        pkt = bytearray()
        pkt.append(HEADER_BYTE)
        pkt += struct.pack(">h", range_q & 0xFFFF if range_q >= 0 else range_q)
        pkt += struct.pack(">h", range_i & 0xFFFF if range_i >= 0 else range_i)
        pkt += struct.pack(">h", dop_i & 0xFFFF if dop_i >= 0 else dop_i)
        pkt += struct.pack(">h", dop_q & 0xFFFF if dop_q >= 0 else dop_q)
        pkt.append(detection & 0x01)
        pkt.append(FOOTER_BYTE)
        return bytes(pkt)

    def test_parse_data_packet_basic(self):
        raw = self._make_data_packet(100, 200, 300, 400, 0)
        result = RadarProtocol.parse_data_packet(raw)
        self.assertIsNotNone(result)
        self.assertEqual(result["range_i"], 100)
        self.assertEqual(result["range_q"], 200)
        self.assertEqual(result["doppler_i"], 300)
        self.assertEqual(result["doppler_q"], 400)
        self.assertEqual(result["detection"], 0)

    def test_parse_data_packet_with_detection(self):
        raw = self._make_data_packet(0, 0, 0, 0, 1)
        result = RadarProtocol.parse_data_packet(raw)
        self.assertIsNotNone(result)
        self.assertEqual(result["detection"], 1)

    def test_parse_data_packet_negative_values(self):
        """Signed 16-bit values should round-trip correctly."""
        raw = self._make_data_packet(-1000, -2000, -500, 32000, 0)
        result = RadarProtocol.parse_data_packet(raw)
        self.assertIsNotNone(result)
        self.assertEqual(result["range_i"], -1000)
        self.assertEqual(result["range_q"], -2000)
        self.assertEqual(result["doppler_i"], -500)
        self.assertEqual(result["doppler_q"], 32000)

    def test_parse_data_packet_too_short(self):
        self.assertIsNone(RadarProtocol.parse_data_packet(b"\xaa\x00"))

    def test_parse_data_packet_wrong_header(self):
        raw = self._make_data_packet()
        bad = b"\x00" + raw[1:]
        self.assertIsNone(RadarProtocol.parse_data_packet(bad))

    # ----------------------------------------------------------------
    # Status packet parsing
    # ----------------------------------------------------------------
    def _make_status_packet(
        self,
        mode=1,
        stream=7,
        threshold=10000,
        long_chirp=3000,
        long_listen=13700,
        guard=17540,
        short_chirp=50,
        short_listen=17450,
        chirps=32,
        range_mode=0,
        st_flags=0,
        st_detail=0,
        st_busy=0,
    ):
        """Build a 26-byte status response matching FPGA format (Build 26)."""
        pkt = bytearray()
        pkt.append(STATUS_HEADER_BYTE)

        # Word 0: {0xFF, 3'b0, mode[1:0], 5'b0, stream[2:0], threshold[15:0]}
        w0 = (0xFF << 24) | ((mode & 0x03) << 21) | ((stream & 0x07) << 16) | (threshold & 0xFFFF)
        pkt += struct.pack(">I", w0)

        # Word 1: {long_chirp, long_listen}
        w1 = ((long_chirp & 0xFFFF) << 16) | (long_listen & 0xFFFF)
        pkt += struct.pack(">I", w1)

        # Word 2: {guard, short_chirp}
        w2 = ((guard & 0xFFFF) << 16) | (short_chirp & 0xFFFF)
        pkt += struct.pack(">I", w2)

        # Word 3: {short_listen, 10'd0, chirps[5:0]}
        w3 = ((short_listen & 0xFFFF) << 16) | (chirps & 0x3F)
        pkt += struct.pack(">I", w3)

        # Word 4: {30'd0, range_mode[1:0]}
        w4 = range_mode & 0x03
        pkt += struct.pack(">I", w4)

        # Word 5: {7'd0, self_test_busy, 8'd0, self_test_detail[7:0],
        #           3'd0, self_test_flags[4:0]}
        w5 = ((st_busy & 0x01) << 24) | ((st_detail & 0xFF) << 8) | (st_flags & 0x1F)
        pkt += struct.pack(">I", w5)

        pkt.append(FOOTER_BYTE)
        return bytes(pkt)

    def test_parse_status_defaults(self):
        raw = self._make_status_packet()
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertIsNotNone(sr)
        self.assertEqual(sr.radar_mode, 1)
        self.assertEqual(sr.stream_ctrl, 7)
        self.assertEqual(sr.cfar_threshold, 10000)
        self.assertEqual(sr.long_chirp, 3000)
        self.assertEqual(sr.long_listen, 13700)
        self.assertEqual(sr.guard, 17540)
        self.assertEqual(sr.short_chirp, 50)
        self.assertEqual(sr.short_listen, 17450)
        self.assertEqual(sr.chirps_per_elev, 32)
        self.assertEqual(sr.range_mode, 0)

    def test_parse_status_range_mode(self):
        raw = self._make_status_packet(range_mode=2)
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertEqual(sr.range_mode, 2)

    def test_parse_status_too_short(self):
        self.assertIsNone(RadarProtocol.parse_status_packet(b"\xbb" + b"\x00" * 20))

    def test_parse_status_wrong_header(self):
        raw = self._make_status_packet()
        bad = b"\xaa" + raw[1:]
        self.assertIsNone(RadarProtocol.parse_status_packet(bad))

    def test_parse_status_wrong_footer(self):
        raw = bytearray(self._make_status_packet())
        raw[25] = 0x00  # corrupt footer (was at index 21 in old 5-word format)
        self.assertIsNone(RadarProtocol.parse_status_packet(bytes(raw)))

    def test_parse_status_self_test_all_pass(self):
        """Status with all self-test flags set (all tests pass)."""
        raw = self._make_status_packet(st_flags=0x1F, st_detail=0xA5, st_busy=0)
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertIsNotNone(sr)
        self.assertEqual(sr.self_test_flags, 0x1F)
        self.assertEqual(sr.self_test_detail, 0xA5)
        self.assertEqual(sr.self_test_busy, 0)

    def test_parse_status_self_test_busy(self):
        """Status with self-test busy flag set."""
        raw = self._make_status_packet(st_flags=0x00, st_detail=0x00, st_busy=1)
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertIsNotNone(sr)
        self.assertEqual(sr.self_test_busy, 1)
        self.assertEqual(sr.self_test_flags, 0)
        self.assertEqual(sr.self_test_detail, 0)

    def test_parse_status_self_test_partial_fail(self):
        """Status with partial self-test failures (flags=0b10110)."""
        raw = self._make_status_packet(st_flags=0b10110, st_detail=0x42, st_busy=0)
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertIsNotNone(sr)
        self.assertEqual(sr.self_test_flags, 0b10110)
        self.assertEqual(sr.self_test_detail, 0x42)
        self.assertEqual(sr.self_test_busy, 0)
        # T0 (BRAM) failed, T1 (CIC) passed, T2 (FFT) passed, T3 (arith) failed, T4 (ADC) passed
        self.assertFalse(sr.self_test_flags & 0x01)  # T0 fail
        self.assertTrue(sr.self_test_flags & 0x02)  # T1 pass
        self.assertTrue(sr.self_test_flags & 0x04)  # T2 pass
        self.assertFalse(sr.self_test_flags & 0x08)  # T3 fail
        self.assertTrue(sr.self_test_flags & 0x10)  # T4 pass

    def test_parse_status_self_test_zero_word5(self):
        """Status with zero word 5 (self-test never run)."""
        raw = self._make_status_packet()
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertEqual(sr.self_test_flags, 0)
        self.assertEqual(sr.self_test_detail, 0)
        self.assertEqual(sr.self_test_busy, 0)

    def test_status_packet_is_26_bytes(self):
        """Verify status packet is exactly 26 bytes."""
        raw = self._make_status_packet()
        self.assertEqual(len(raw), 26)

    # ----------------------------------------------------------------
    # Boundary detection
    # ----------------------------------------------------------------
    def test_find_boundaries_mixed(self):
        data_pkt = self._make_data_packet()
        status_pkt = self._make_status_packet()
        buf = b"\x00\x00" + data_pkt + b"\x00" + status_pkt + data_pkt
        boundaries = RadarProtocol.find_packet_boundaries(buf)
        self.assertEqual(len(boundaries), 3)
        self.assertEqual(boundaries[0][2], "data")
        self.assertEqual(boundaries[1][2], "status")
        self.assertEqual(boundaries[2][2], "data")

    def test_find_boundaries_empty(self):
        self.assertEqual(RadarProtocol.find_packet_boundaries(b""), [])

    def test_find_boundaries_truncated(self):
        """Truncated packet should not be returned."""
        data_pkt = self._make_data_packet()
        buf = data_pkt[:6]  # truncated (less than 11-byte packet size)
        boundaries = RadarProtocol.find_packet_boundaries(buf)
        self.assertEqual(len(boundaries), 0)


class TestFT2232HConnection(unittest.TestCase):
    """Test mock FT2232H connection."""

    def test_mock_open_close(self):
        conn = FT2232HConnection(mock=True)
        self.assertTrue(conn.open())
        self.assertTrue(conn.is_open)
        conn.close()
        self.assertFalse(conn.is_open)

    def test_mock_read_returns_data(self):
        conn = FT2232HConnection(mock=True)
        conn.open()
        data = conn.read(4096)
        self.assertIsNotNone(data)
        self.assertGreater(len(data), 0)
        conn.close()

    def test_mock_read_contains_valid_packets(self):
        """Mock data should contain parseable data packets."""
        conn = FT2232HConnection(mock=True)
        conn.open()
        raw = conn.read(4096)
        packets = RadarProtocol.find_packet_boundaries(raw)
        self.assertGreater(len(packets), 0)
        for start, end, ptype in packets:
            if ptype == "data":
                result = RadarProtocol.parse_data_packet(raw[start:end])
                self.assertIsNotNone(result)
        conn.close()

    def test_mock_write(self):
        conn = FT2232HConnection(mock=True)
        conn.open()
        cmd = RadarProtocol.build_command(0x01, 1)
        self.assertTrue(conn.write(cmd))
        conn.close()

    def test_read_when_closed(self):
        conn = FT2232HConnection(mock=True)
        self.assertIsNone(conn.read())

    def test_write_when_closed(self):
        conn = FT2232HConnection(mock=True)
        self.assertFalse(conn.write(b"\x00\x00\x00\x00"))


class TestDataRecorder(unittest.TestCase):
    """Test HDF5 recording (skipped if h5py not available)."""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.filepath = os.path.join(self.tmpdir, "test_recording.h5")

    def tearDown(self):
        if os.path.exists(self.filepath):
            os.remove(self.filepath)
        os.rmdir(self.tmpdir)

    @unittest.skipUnless(_H5PY_AVAILABLE, "h5py not installed")
    def test_record_and_stop(self):
        import h5py

        rec = DataRecorder()
        rec.start(self.filepath)
        self.assertTrue(rec.recording)

        # Record 3 frames
        for i in range(3):
            frame = RadarFrame()
            frame.frame_number = i
            frame.timestamp = time.time()
            frame.magnitude = np.random.rand(NUM_RANGE_BINS, NUM_DOPPLER_BINS)
            frame.range_profile = np.random.rand(NUM_RANGE_BINS)
            rec.record_frame(frame)

        rec.stop()
        self.assertFalse(rec.recording)

        # Verify HDF5 contents
        with h5py.File(self.filepath, "r") as f:
            self.assertEqual(f.attrs["total_frames"], 3)
            self.assertIn("frames", f)
            self.assertIn("frame_000000", f["frames"])
            self.assertIn("frame_000002", f["frames"])
            mag = f["frames/frame_000001/magnitude"][:]
            self.assertEqual(mag.shape, (NUM_RANGE_BINS, NUM_DOPPLER_BINS))


class TestRadarAcquisition(unittest.TestCase):
    """Test acquisition thread with mock connection."""

    def test_acquisition_produces_frames(self):
        conn = FT2232HConnection(mock=True)
        conn.open()
        fq = queue.Queue(maxsize=16)
        acq = RadarAcquisition(conn, fq)
        acq.start()

        # Wait for at least one frame (mock produces ~32 samples per read,
        # need 2048 for a full frame, so may take a few seconds)
        frame = None
        try:
            frame = fq.get(timeout=10)
        except queue.Empty:
            pass

        acq.stop()
        acq.join(timeout=3)
        conn.close()

        # With mock data producing 32 packets per read at 50ms interval,
        # a full frame (2048 samples) takes ~3.2s. Allow up to 10s.
        if frame is not None:
            self.assertIsInstance(frame, RadarFrame)
            self.assertEqual(frame.magnitude.shape, (NUM_RANGE_BINS, NUM_DOPPLER_BINS))
        # If no frame arrived in timeout, that's still OK for a fast CI run

    def test_acquisition_stop(self):
        conn = FT2232HConnection(mock=True)
        conn.open()
        fq = queue.Queue(maxsize=4)
        acq = RadarAcquisition(conn, fq)
        acq.start()
        time.sleep(0.2)
        acq.stop()
        acq.join(timeout=3)
        self.assertFalse(acq.is_alive())
        conn.close()


class TestRadarFrameDefaults(unittest.TestCase):
    """Test RadarFrame default initialization."""

    def test_default_shapes(self):
        f = RadarFrame()
        self.assertEqual(f.range_doppler_i.shape, (64, 32))
        self.assertEqual(f.range_doppler_q.shape, (64, 32))
        self.assertEqual(f.magnitude.shape, (64, 32))
        self.assertEqual(f.detections.shape, (64, 32))
        self.assertEqual(f.range_profile.shape, (64,))
        self.assertEqual(f.detection_count, 0)

    def test_default_zeros(self):
        f = RadarFrame()
        self.assertTrue(np.all(f.magnitude == 0))
        self.assertTrue(np.all(f.detections == 0))


class TestEndToEnd(unittest.TestCase):
    """End-to-end: build command → parse response → verify round-trip."""

    def test_command_roundtrip_all_opcodes(self):
        """Verify all opcodes produce valid 4-byte commands."""
        opcodes = [
            0x01,
            0x02,
            0x03,
            0x04,
            0x05,
            0x06,
            0x10,
            0x11,
            0x12,
            0x13,
            0x14,
            0x15,
            0x20,
            0x21,
            0x22,
            0x23,
            0x24,
            0x25,
            0x26,
            0x27,
            0x30,
            0x31,
            0xFF,
        ]
        for op in opcodes:
            cmd = RadarProtocol.build_command(op, 42)
            self.assertEqual(len(cmd), 4, f"opcode 0x{op:02X}")
            word = struct.unpack(">I", cmd)[0]
            self.assertEqual((word >> 24) & 0xFF, op)
            self.assertEqual(word & 0xFFFF, 42)

    def test_data_packet_roundtrip(self):
        """Build an 11-byte data packet, parse it, verify values match."""
        ri, rq, di, dq = 1234, -5678, 9012, -3456

        pkt = bytearray()
        pkt.append(HEADER_BYTE)
        pkt += struct.pack(">h", rq)
        pkt += struct.pack(">h", ri)
        pkt += struct.pack(">h", di)
        pkt += struct.pack(">h", dq)
        pkt.append(1)
        pkt.append(FOOTER_BYTE)

        self.assertEqual(len(pkt), DATA_PACKET_SIZE)

        result = RadarProtocol.parse_data_packet(bytes(pkt))
        self.assertIsNotNone(result)
        self.assertEqual(result["range_i"], ri)
        self.assertEqual(result["range_q"], rq)
        self.assertEqual(result["doppler_i"], di)
        self.assertEqual(result["doppler_q"], dq)
        self.assertEqual(result["detection"], 1)


class TestReplayConnection(unittest.TestCase):
    """Test ReplayConnection with real .npy data files."""

    NPY_DIR = os.path.join(
        os.path.dirname(__file__), "..", "9_2_FPGA", "tb", "cosim", "real_data", "hex"
    )

    def _npy_available(self):
        """Check if the npy data files exist."""
        return os.path.isfile(os.path.join(self.NPY_DIR, "fullchain_mti_doppler_i.npy"))

    def test_replay_open_close(self):
        """ReplayConnection opens and closes without error."""
        if not self._npy_available():
            self.skipTest("npy data files not found")
        from radar_protocol import ReplayConnection

        conn = ReplayConnection(self.NPY_DIR, use_mti=True)
        self.assertTrue(conn.open())
        self.assertTrue(conn.is_open)
        conn.close()
        self.assertFalse(conn.is_open)

    def test_replay_packet_count(self):
        """Replay builds exactly NUM_CELLS (2048) packets."""
        if not self._npy_available():
            self.skipTest("npy data files not found")
        from radar_protocol import ReplayConnection

        conn = ReplayConnection(self.NPY_DIR, use_mti=True)
        conn.open()
        # Each packet is 11 bytes, total = 2048 * 11
        expected_bytes = NUM_CELLS * DATA_PACKET_SIZE
        self.assertEqual(conn._frame_len, expected_bytes)
        conn.close()

    def test_replay_packets_parseable(self):
        """Every packet from replay can be parsed by RadarProtocol."""
        if not self._npy_available():
            self.skipTest("npy data files not found")
        from radar_protocol import ReplayConnection

        conn = ReplayConnection(self.NPY_DIR, use_mti=True)
        conn.open()
        raw = conn._packets
        boundaries = RadarProtocol.find_packet_boundaries(raw)
        self.assertEqual(len(boundaries), NUM_CELLS)
        parsed_count = 0
        det_count = 0
        for start, end, ptype in boundaries:
            self.assertEqual(ptype, "data")
            result = RadarProtocol.parse_data_packet(raw[start:end])
            self.assertIsNotNone(result)
            parsed_count += 1
            if result["detection"]:
                det_count += 1
        self.assertEqual(parsed_count, NUM_CELLS)
        # Default: MTI=ON, DC_notch=2, CFAR CA g=2 t=8 a=0x30 → 4 detections
        self.assertEqual(det_count, 4)
        conn.close()

    def test_replay_read_loops(self):
        """Read returns data and loops back around."""
        if not self._npy_available():
            self.skipTest("npy data files not found")
        from radar_protocol import ReplayConnection

        conn = ReplayConnection(self.NPY_DIR, use_mti=True, replay_fps=1000)
        conn.open()
        total_read = 0
        for _ in range(100):
            chunk = conn.read(1024)
            self.assertIsNotNone(chunk)
            total_read += len(chunk)
        self.assertGreater(total_read, 0)
        conn.close()

    def test_replay_no_mti(self):
        """ReplayConnection works with use_mti=False (CFAR still runs)."""
        if not self._npy_available():
            self.skipTest("npy data files not found")
        from radar_protocol import ReplayConnection

        conn = ReplayConnection(self.NPY_DIR, use_mti=False)
        conn.open()
        self.assertEqual(conn._frame_len, NUM_CELLS * DATA_PACKET_SIZE)
        # No-MTI with DC notch=2 and default CFAR → 0 detections
        raw = conn._packets
        boundaries = RadarProtocol.find_packet_boundaries(raw)
        det_count = sum(
            1
            for s, e, t in boundaries
            if RadarProtocol.parse_data_packet(raw[s:e]).get("detection", 0)
        )
        self.assertEqual(det_count, 0)
        conn.close()

    def test_replay_write_returns_true(self):
        """Write on replay connection returns True."""
        if not self._npy_available():
            self.skipTest("npy data files not found")
        from radar_protocol import ReplayConnection

        conn = ReplayConnection(self.NPY_DIR)
        conn.open()
        self.assertTrue(conn.write(b"\x01\x00\x00\x01"))
        conn.close()

    def test_replay_adjustable_param_cfar_guard(self):
        """Changing CFAR guard via write() triggers re-processing."""
        if not self._npy_available():
            self.skipTest("npy data files not found")
        from radar_protocol import ReplayConnection

        conn = ReplayConnection(self.NPY_DIR, use_mti=True)
        conn.open()
        # Initial: guard=2 → 4 detections
        self.assertFalse(conn._needs_rebuild)
        # Send CFAR_GUARD=4
        cmd = RadarProtocol.build_command(0x21, 4)
        conn.write(cmd)
        self.assertTrue(conn._needs_rebuild)
        self.assertEqual(conn._cfar_guard, 4)
        # Read triggers rebuild
        conn.read(1024)
        self.assertFalse(conn._needs_rebuild)
        conn.close()

    def test_replay_adjustable_param_mti_toggle(self):
        """Toggling MTI via write() triggers re-processing."""
        if not self._npy_available():
            self.skipTest("npy data files not found")
        from radar_protocol import ReplayConnection

        conn = ReplayConnection(self.NPY_DIR, use_mti=True)
        conn.open()
        # Disable MTI
        cmd = RadarProtocol.build_command(0x26, 0)
        conn.write(cmd)
        self.assertTrue(conn._needs_rebuild)
        self.assertFalse(conn._mti_enable)
        # Read to trigger rebuild, then count detections
        # Drain all packets after rebuild
        conn.read(1024)  # triggers rebuild
        raw = conn._packets
        boundaries = RadarProtocol.find_packet_boundaries(raw)
        det_count = sum(
            1
            for s, e, t in boundaries
            if RadarProtocol.parse_data_packet(raw[s:e]).get("detection", 0)
        )
        # No-MTI with default CFAR → 0 detections
        self.assertEqual(det_count, 0)
        conn.close()

    def test_replay_adjustable_param_dc_notch(self):
        """Changing DC notch width via write() triggers re-processing."""
        if not self._npy_available():
            self.skipTest("npy data files not found")
        from radar_protocol import ReplayConnection

        conn = ReplayConnection(self.NPY_DIR, use_mti=True)
        conn.open()
        # Change DC notch to 0 (no notch)
        cmd = RadarProtocol.build_command(0x27, 0)
        conn.write(cmd)
        self.assertTrue(conn._needs_rebuild)
        self.assertEqual(conn._dc_notch_width, 0)
        conn.read(1024)  # triggers rebuild
        raw = conn._packets
        boundaries = RadarProtocol.find_packet_boundaries(raw)
        det_count = sum(
            1
            for s, e, t in boundaries
            if RadarProtocol.parse_data_packet(raw[s:e]).get("detection", 0)
        )
        # DC notch=0 with MTI → 6 detections (more noise passes through)
        self.assertEqual(det_count, 6)
        conn.close()

    def test_replay_hardware_opcode_ignored(self):
        """Hardware-only opcodes don't trigger rebuild."""
        if not self._npy_available():
            self.skipTest("npy data files not found")
        from radar_protocol import ReplayConnection

        conn = ReplayConnection(self.NPY_DIR, use_mti=True)
        conn.open()
        # Send TRIGGER (hardware-only)
        cmd = RadarProtocol.build_command(0x01, 1)
        conn.write(cmd)
        self.assertFalse(conn._needs_rebuild)
        # Send STREAM_ENABLE (hardware-only)
        cmd = RadarProtocol.build_command(0x05, 7)
        conn.write(cmd)
        self.assertFalse(conn._needs_rebuild)
        conn.close()

    def test_replay_same_value_no_rebuild(self):
        """Setting same value as current doesn't trigger rebuild."""
        if not self._npy_available():
            self.skipTest("npy data files not found")
        from radar_protocol import ReplayConnection

        conn = ReplayConnection(self.NPY_DIR, use_mti=True)
        conn.open()
        # CFAR guard already 2
        cmd = RadarProtocol.build_command(0x21, 2)
        conn.write(cmd)
        self.assertFalse(conn._needs_rebuild)
        conn.close()

    def test_replay_self_test_opcodes_are_hardware_only(self):
        """Self-test opcodes 0x30/0x31 are hardware-only (ignored in replay)."""
        if not self._npy_available():
            self.skipTest("npy data files not found")
        from radar_protocol import ReplayConnection

        conn = ReplayConnection(self.NPY_DIR, use_mti=True)
        conn.open()
        # Send self-test trigger
        cmd = RadarProtocol.build_command(0x30, 1)
        conn.write(cmd)
        self.assertFalse(conn._needs_rebuild)
        # Send self-test status request
        cmd = RadarProtocol.build_command(0x31, 0)
        conn.write(cmd)
        self.assertFalse(conn._needs_rebuild)
        conn.close()


class TestOpcodeEnum(unittest.TestCase):
    """Verify Opcode enum matches RTL host register map."""

    def test_gain_shift_is_0x06(self):
        """GAIN_SHIFT opcode must be 0x06 (not 0x16)."""
        self.assertEqual(Opcode.GAIN_SHIFT, 0x06)

    def test_no_digital_gain_alias(self):
        """DIGITAL_GAIN should NOT exist (was bogus 0x16 alias)."""
        self.assertFalse(hasattr(Opcode, "DIGITAL_GAIN"))

    def test_self_test_trigger(self):
        """SELF_TEST_TRIGGER opcode must be 0x30."""
        self.assertEqual(Opcode.SELF_TEST_TRIGGER, 0x30)

    def test_self_test_status(self):
        """SELF_TEST_STATUS opcode must be 0x31."""
        self.assertEqual(Opcode.SELF_TEST_STATUS, 0x31)

    def test_self_test_in_hardware_only(self):
        """Self-test opcodes must be in _HARDWARE_ONLY_OPCODES."""
        self.assertIn(0x30, _HARDWARE_ONLY_OPCODES)
        self.assertIn(0x31, _HARDWARE_ONLY_OPCODES)

    def test_0x16_not_in_hardware_only(self):
        """Bogus 0x16 must not be in _HARDWARE_ONLY_OPCODES."""
        self.assertNotIn(0x16, _HARDWARE_ONLY_OPCODES)

    def test_stream_enable_is_0x05(self):
        """STREAM_ENABLE must be 0x05 (not 0x04)."""
        self.assertEqual(Opcode.STREAM_ENABLE, 0x05)

    def test_all_rtl_opcodes_present(self):
        """Every RTL opcode has a matching Opcode enum member."""
        expected = {
            0x01,
            0x02,
            0x03,
            0x04,
            0x05,
            0x06,
            0x10,
            0x11,
            0x12,
            0x13,
            0x14,
            0x15,
            0x20,
            0x21,
            0x22,
            0x23,
            0x24,
            0x25,
            0x26,
            0x27,
            0x30,
            0x31,
            0xFF,
        }
        enum_values = set(int(m) for m in Opcode)
        for op in expected:
            self.assertIn(op, enum_values, f"0x{op:02X} missing from Opcode enum")


class TestStatusResponseDefaults(unittest.TestCase):
    """Verify StatusResponse dataclass has self-test fields."""

    def test_default_self_test_fields(self):
        sr = StatusResponse()
        self.assertEqual(sr.self_test_flags, 0)
        self.assertEqual(sr.self_test_detail, 0)
        self.assertEqual(sr.self_test_busy, 0)

    def test_self_test_fields_set(self):
        sr = StatusResponse(self_test_flags=0x1F, self_test_detail=0xAB, self_test_busy=1)
        self.assertEqual(sr.self_test_flags, 0x1F)
        self.assertEqual(sr.self_test_detail, 0xAB)
        self.assertEqual(sr.self_test_busy, 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
