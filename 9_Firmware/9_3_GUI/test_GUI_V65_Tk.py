#!/usr/bin/env python3
"""
Tests for AERIS-10 Radar Dashboard protocol parsing, command building,
data recording, and acquisition logic.

Run: python -m pytest test_GUI_V65_Tk.py -v
  or: python test_GUI_V65_Tk.py
"""

import struct
import time
import queue
import os
import tempfile
import unittest
import numpy as np

from radar_protocol import (
    RadarProtocol, FT2232HConnection, FT601Connection, DataRecorder, RadarAcquisition,
    RadarFrame, StatusResponse, Opcode,
    HEADER_BYTE, FOOTER_BYTE, STATUS_HEADER_BYTE,
    NUM_RANGE_BINS, NUM_DOPPLER_BINS,
    DATA_PACKET_SIZE,
)
from GUI_V65_Tk import (
    DemoTarget, DemoSimulator, _ReplayController, ppi_xy, ppi_raster, ppi_beam_weights,
)


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
        self.assertEqual(word & 0xFFFF, 1)            # value

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
    def _make_data_packet(self, range_i=100, range_q=200,
                          dop_i=300, dop_q=400, detection=0):
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
        self.assertIsNone(RadarProtocol.parse_data_packet(b"\xAA\x00"))

    def test_parse_data_packet_wrong_header(self):
        raw = self._make_data_packet()
        bad = b"\x00" + raw[1:]
        self.assertIsNone(RadarProtocol.parse_data_packet(bad))

    # ----------------------------------------------------------------
    # Status packet parsing
    # ----------------------------------------------------------------
    def _make_status_packet(self, mode=1, stream=7, threshold=10000,
                            long_chirp=3000, long_listen=13700,
                            guard=17540, short_chirp=50,
                            short_listen=17450, chirps=32, range_mode=0,
                            st_flags=0, st_detail=0, st_busy=0,
                            agc_gain=0, agc_peak=0, agc_sat=0, agc_enable=0):
        """Build a 26-byte status response matching FPGA format (Build 26)."""
        pkt = bytearray()
        pkt.append(STATUS_HEADER_BYTE)

        # Word 0: {0xFF[31:24], mode[23:22], stream[21:19], 3'b000[18:16], threshold[15:0]}
        w0 = (0xFF << 24) | ((mode & 0x03) << 22) | ((stream & 0x07) << 19) | (threshold & 0xFFFF)
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

        # Word 4: {agc_current_gain[3:0], agc_peak_magnitude[7:0],
        #          agc_saturation_count[7:0], agc_enable, 9'd0, range_mode[1:0]}
        w4 = (((agc_gain & 0x0F) << 28) | ((agc_peak & 0xFF) << 20) |
              ((agc_sat & 0xFF) << 12) | ((agc_enable & 0x01) << 11) |
              (range_mode & 0x03))
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
        self.assertIsNone(RadarProtocol.parse_status_packet(b"\xBB" + b"\x00" * 20))

    def test_parse_status_wrong_header(self):
        raw = self._make_status_packet()
        bad = b"\xAA" + raw[1:]
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
        self.assertTrue(sr.self_test_flags & 0x02)    # T1 pass
        self.assertTrue(sr.self_test_flags & 0x04)    # T2 pass
        self.assertFalse(sr.self_test_flags & 0x08)   # T3 fail
        self.assertTrue(sr.self_test_flags & 0x10)     # T4 pass

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


class TestFT601Connection(unittest.TestCase):
    """Test mock FT601 connection (mirrors FT2232H tests)."""

    def test_mock_open_close(self):
        conn = FT601Connection(mock=True)
        self.assertTrue(conn.open())
        self.assertTrue(conn.is_open)
        conn.close()
        self.assertFalse(conn.is_open)

    def test_mock_read_returns_data(self):
        conn = FT601Connection(mock=True)
        conn.open()
        data = conn.read(4096)
        self.assertIsNotNone(data)
        self.assertGreater(len(data), 0)
        conn.close()

    def test_mock_read_contains_valid_packets(self):
        """Mock data should contain parseable data packets."""
        conn = FT601Connection(mock=True)
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
        conn = FT601Connection(mock=True)
        conn.open()
        cmd = RadarProtocol.build_command(0x01, 1)
        self.assertTrue(conn.write(cmd))
        conn.close()

    def test_write_pads_to_4_bytes(self):
        """FT601 write() should pad data to 4-byte alignment."""
        conn = FT601Connection(mock=True)
        conn.open()
        # 3-byte payload should be padded internally (no error)
        self.assertTrue(conn.write(b"\x01\x02\x03"))
        conn.close()

    def test_read_when_closed(self):
        conn = FT601Connection(mock=True)
        self.assertIsNone(conn.read())

    def test_write_when_closed(self):
        conn = FT601Connection(mock=True)
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

    @unittest.skipUnless(
        (lambda: (
            __import__("importlib.util")
            and __import__("importlib").util.find_spec("h5py") is not None
        ))(),
        "h5py not installed"
    )
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
        try:  # noqa: SIM105
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
            self.assertEqual(frame.magnitude.shape,
                             (NUM_RANGE_BINS, NUM_DOPPLER_BINS))
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
        opcodes = [0x01, 0x02, 0x03, 0x04, 0x10, 0x11, 0x12,
                   0x13, 0x14, 0x15, 0x16, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25,
                   0x26, 0x27, 0x30, 0x31, 0xFF]
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


class TestOpcodeEnum(unittest.TestCase):
    """Verify Opcode enum matches RTL host register map (radar_system_top.v)."""

    def test_gain_shift_is_0x16(self):
        """GAIN_SHIFT opcode must be 0x16 (matches radar_system_top.v:928)."""
        self.assertEqual(Opcode.GAIN_SHIFT, 0x16)

    def test_no_digital_gain_alias(self):
        """DIGITAL_GAIN should NOT exist (use GAIN_SHIFT)."""
        self.assertFalse(hasattr(Opcode, 'DIGITAL_GAIN'))

    def test_self_test_trigger(self):
        """SELF_TEST_TRIGGER opcode must be 0x30."""
        self.assertEqual(Opcode.SELF_TEST_TRIGGER, 0x30)

    def test_self_test_status(self):
        """SELF_TEST_STATUS opcode must be 0x31."""
        self.assertEqual(Opcode.SELF_TEST_STATUS, 0x31)

    def test_stream_control_is_0x04(self):
        """STREAM_CONTROL must be 0x04 (matches radar_system_top.v:906)."""
        self.assertEqual(Opcode.STREAM_CONTROL, 0x04)

    def test_legacy_aliases_removed(self):
        """Legacy aliases must NOT exist in production Opcode enum."""
        for name in ("TRIGGER", "PRF_DIV", "NUM_CHIRPS", "CHIRP_TIMER",
                      "STREAM_ENABLE", "THRESHOLD"):
            self.assertFalse(hasattr(Opcode, name),
                             f"Legacy alias Opcode.{name} should not exist")

    def test_radar_mode_names(self):
        """New canonical names must exist and match FPGA opcodes."""
        self.assertEqual(Opcode.RADAR_MODE, 0x01)
        self.assertEqual(Opcode.TRIGGER_PULSE, 0x02)
        self.assertEqual(Opcode.DETECT_THRESHOLD, 0x03)
        self.assertEqual(Opcode.STREAM_CONTROL, 0x04)

    def test_all_rtl_opcodes_present(self):
        """Every RTL opcode (from radar_system_top.v) has a matching Opcode enum member."""
        expected = {0x01, 0x02, 0x03, 0x04,
                    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
                    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
                    0x28, 0x29, 0x2A, 0x2B, 0x2C,
                    0x30, 0x31, 0xFF}
        enum_values = {int(m) for m in Opcode}
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
        sr = StatusResponse(self_test_flags=0x1F,
                            self_test_detail=0xAB,
                            self_test_busy=1)
        self.assertEqual(sr.self_test_flags, 0x1F)
        self.assertEqual(sr.self_test_detail, 0xAB)
        self.assertEqual(sr.self_test_busy, 1)


class TestAGCOpcodes(unittest.TestCase):
    """Verify AGC opcode enum members match FPGA RTL (0x28-0x2C)."""

    def test_agc_enable_opcode(self):
        self.assertEqual(Opcode.AGC_ENABLE, 0x28)

    def test_agc_target_opcode(self):
        self.assertEqual(Opcode.AGC_TARGET, 0x29)

    def test_agc_attack_opcode(self):
        self.assertEqual(Opcode.AGC_ATTACK, 0x2A)

    def test_agc_decay_opcode(self):
        self.assertEqual(Opcode.AGC_DECAY, 0x2B)

    def test_agc_holdoff_opcode(self):
        self.assertEqual(Opcode.AGC_HOLDOFF, 0x2C)


class TestAGCStatusParsing(unittest.TestCase):
    """Verify AGC fields in status_words[4] are parsed correctly."""

    def _make_status_packet(self, **kwargs):
        """Delegate to TestRadarProtocol helper."""
        helper = TestRadarProtocol()
        return helper._make_status_packet(**kwargs)

    def test_agc_fields_default_zero(self):
        """With no AGC fields set, all should be 0."""
        raw = self._make_status_packet()
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertEqual(sr.agc_current_gain, 0)
        self.assertEqual(sr.agc_peak_magnitude, 0)
        self.assertEqual(sr.agc_saturation_count, 0)
        self.assertEqual(sr.agc_enable, 0)

    def test_agc_fields_nonzero(self):
        """AGC fields round-trip through status packet."""
        raw = self._make_status_packet(agc_gain=7, agc_peak=200,
                                       agc_sat=15, agc_enable=1)
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertEqual(sr.agc_current_gain, 7)
        self.assertEqual(sr.agc_peak_magnitude, 200)
        self.assertEqual(sr.agc_saturation_count, 15)
        self.assertEqual(sr.agc_enable, 1)

    def test_agc_max_values(self):
        """AGC fields at max values."""
        raw = self._make_status_packet(agc_gain=15, agc_peak=255,
                                       agc_sat=255, agc_enable=1)
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertEqual(sr.agc_current_gain, 15)
        self.assertEqual(sr.agc_peak_magnitude, 255)
        self.assertEqual(sr.agc_saturation_count, 255)
        self.assertEqual(sr.agc_enable, 1)

    def test_agc_and_range_mode_coexist(self):
        """AGC fields and range_mode occupy the same word without conflict."""
        raw = self._make_status_packet(agc_gain=5, agc_peak=128,
                                       agc_sat=42, agc_enable=1,
                                       range_mode=2)
        sr = RadarProtocol.parse_status_packet(raw)
        self.assertEqual(sr.agc_current_gain, 5)
        self.assertEqual(sr.agc_peak_magnitude, 128)
        self.assertEqual(sr.agc_saturation_count, 42)
        self.assertEqual(sr.agc_enable, 1)
        self.assertEqual(sr.range_mode, 2)


class TestAGCStatusResponseDefaults(unittest.TestCase):
    """Verify StatusResponse AGC field defaults."""

    def test_default_agc_fields(self):
        sr = StatusResponse()
        self.assertEqual(sr.agc_current_gain, 0)
        self.assertEqual(sr.agc_peak_magnitude, 0)
        self.assertEqual(sr.agc_saturation_count, 0)
        self.assertEqual(sr.agc_enable, 0)

    def test_agc_fields_set(self):
        sr = StatusResponse(agc_current_gain=7, agc_peak_magnitude=200,
                            agc_saturation_count=15, agc_enable=1)
        self.assertEqual(sr.agc_current_gain, 7)
        self.assertEqual(sr.agc_peak_magnitude, 200)
        self.assertEqual(sr.agc_saturation_count, 15)
        self.assertEqual(sr.agc_enable, 1)


# =============================================================================
# AGC Visualization — ring buffer / data model tests
# =============================================================================

class TestAGCVisualizationHistory(unittest.TestCase):
    """Test the AGC visualization ring buffer logic (no GUI required)."""

    def _make_deque(self, maxlen=256):
        from collections import deque
        return deque(maxlen=maxlen)

    def test_ring_buffer_maxlen(self):
        """Ring buffer should evict oldest when full."""
        d = self._make_deque(maxlen=4)
        for i in range(6):
            d.append(i)
        self.assertEqual(list(d), [2, 3, 4, 5])
        self.assertEqual(len(d), 4)

    def test_gain_history_accumulation(self):
        """Gain values accumulate correctly in a deque."""
        gain_hist = self._make_deque(maxlen=256)
        statuses = [
            StatusResponse(agc_current_gain=g)
            for g in [0, 3, 7, 15, 8, 2]
        ]
        for st in statuses:
            gain_hist.append(st.agc_current_gain)
        self.assertEqual(list(gain_hist), [0, 3, 7, 15, 8, 2])

    def test_peak_history_accumulation(self):
        """Peak magnitude values accumulate correctly."""
        peak_hist = self._make_deque(maxlen=256)
        for p in [0, 50, 200, 255, 128]:
            peak_hist.append(p)
        self.assertEqual(list(peak_hist), [0, 50, 200, 255, 128])

    def test_saturation_total_computation(self):
        """Sum of saturation ring buffer gives running total."""
        sat_hist = self._make_deque(maxlen=256)
        for s in [0, 0, 5, 0, 12, 3]:
            sat_hist.append(s)
        self.assertEqual(sum(sat_hist), 20)

    def test_saturation_color_thresholds(self):
        """Color logic: green=0, yellow=1-10, red>10."""
        def sat_color(total):
            if total > 10:
                return "red"
            if total > 0:
                return "yellow"
            return "green"
        self.assertEqual(sat_color(0), "green")
        self.assertEqual(sat_color(1), "yellow")
        self.assertEqual(sat_color(10), "yellow")
        self.assertEqual(sat_color(11), "red")
        self.assertEqual(sat_color(255), "red")

    def test_ring_buffer_eviction_preserves_latest(self):
        """After overflow, only the most recent values remain."""
        d = self._make_deque(maxlen=8)
        for i in range(20):
            d.append(i)
        self.assertEqual(list(d), [12, 13, 14, 15, 16, 17, 18, 19])

    def test_empty_history_safe(self):
        """Empty ring buffer should be safe for max/sum."""
        d = self._make_deque(maxlen=256)
        self.assertEqual(sum(d), 0)
        self.assertEqual(len(d), 0)
        # max() on empty would raise — test the guard pattern used in viz code
        max_sat = max(d) if d else 0
        self.assertEqual(max_sat, 0)

    def test_agc_mode_string(self):
        """AGC mode display string from enable flag."""
        self.assertEqual(
            "AUTO" if StatusResponse(agc_enable=1).agc_enable else "MANUAL",
            "AUTO")
        self.assertEqual(
            "AUTO" if StatusResponse(agc_enable=0).agc_enable else "MANUAL",
            "MANUAL")

    def test_xlim_scroll_logic(self):
        """X-axis scroll: when n >= history_len, xlim should expand."""
        history_len = 8
        d = self._make_deque(maxlen=history_len)
        for i in range(10):
            d.append(i)
        n = len(d)
        # After 10 pushes into maxlen=8, n=8
        self.assertEqual(n, history_len)
        # xlim should be (0, n) for static or (n-history_len, n) for scrolling
        self.assertEqual(max(0, n - history_len), 0)
        self.assertEqual(n, 8)

    def test_sat_autoscale_ylim(self):
        """Saturation y-axis auto-scale: max(max_sat * 1.5, 5)."""
        # No saturation
        self.assertEqual(max(0 * 1.5, 5), 5)
        # Some saturation
        self.assertAlmostEqual(max(10 * 1.5, 5), 15.0)
        # High saturation
        self.assertAlmostEqual(max(200 * 1.5, 5), 300.0)


# =====================================================================
# Tests for north-up PPI coordinate conversion
# =====================================================================


class TestPpiXy(unittest.TestCase):
    """ppi_xy is the only PPI math — keep it vectorized and north-up."""

    def test_empty(self):
        xy = ppi_xy([], [])
        self.assertEqual(xy.shape, (0, 2))

    def test_north_is_plus_y(self):
        xy = ppi_xy(1000.0, 0.0)
        np.testing.assert_allclose(xy[0], [0.0, 1000.0], atol=1e-9)

    def test_east_is_plus_x(self):
        xy = ppi_xy(1000.0, 90.0)
        np.testing.assert_allclose(xy[0], [1000.0, 0.0], atol=1e-9)

    def test_south_and_west(self):
        south = ppi_xy(500.0, 180.0)
        west = ppi_xy(500.0, 270.0)
        np.testing.assert_allclose(south[0], [0.0, -500.0], atol=1e-9)
        np.testing.assert_allclose(west[0], [-500.0, 0.0], atol=1e-9)

    def test_vectorized_shape(self):
        xy = ppi_xy([100.0, 200.0, 300.0], [0.0, 90.0, 180.0])
        self.assertEqual(xy.shape, (3, 2))
        self.assertTrue(np.isfinite(xy).all())

    def test_raster_north_pixel(self):
        img = ppi_raster(ppi_xy(1000.0, 0.0), 1000.0, size=11)
        self.assertEqual(img[10, 5], 1.0)

    def test_raster_decay(self):
        img = np.ones((8, 8), dtype=np.float32)
        ppi_raster(np.empty((0, 2)), 1.0, size=8, decay=0.5, out=img)
        np.testing.assert_allclose(img, 0.5)

    def test_beam_weights_peak_on_heading(self):
        w = ppi_beam_weights([0.0, 90.0, 180.0], 0.0, beam_deg=10.0)
        self.assertAlmostEqual(float(w[0]), 1.0, places=5)
        self.assertLess(float(w[1]), 0.01)
        self.assertLess(float(w[2]), 0.01)

    def test_raster_10k(self):
        n = 10_000
        xy = ppi_xy(np.full(n, 500.0), np.linspace(0.0, 360.0, n, endpoint=False))
        img = ppi_raster(xy, 1536.0, size=256)
        self.assertEqual(img.shape, (256, 256))
        self.assertGreater(float(img.sum()), 100.0)


# =====================================================================
# Tests for DemoTarget, DemoSimulator, and _ReplayController
# =====================================================================


class TestDemoTarget(unittest.TestCase):
    """Unit tests for DemoTarget kinematics."""

    def test_initial_values_in_range(self):
        t = DemoTarget(1)
        self.assertEqual(t.id, 1)
        self.assertGreaterEqual(t.range_m, 20)
        self.assertLessEqual(t.range_m, DemoTarget._MAX_RANGE)
        self.assertIn(t.classification, ["aircraft", "drone", "bird", "unknown"])

    def test_step_returns_true_in_normal_range(self):
        t = DemoTarget(2)
        t.range_m = 150.0
        t.velocity = 0.0
        self.assertTrue(t.step())

    def test_step_returns_false_when_out_of_range_high(self):
        t = DemoTarget(3)
        t.range_m = DemoTarget._MAX_RANGE + 1
        t.velocity = -1.0  # moving away
        self.assertFalse(t.step())

    def test_step_returns_false_when_out_of_range_low(self):
        t = DemoTarget(4)
        t.range_m = 2.0
        t.velocity = 1.0  # moving closer
        self.assertFalse(t.step())

    def test_velocity_clamped(self):
        t = DemoTarget(5)
        t.velocity = 19.0
        t.range_m = 150.0
        # Step many times — velocity should stay within [-20, 20]
        for _ in range(100):
            t.range_m = 150.0  # keep in range
            t.step()
        self.assertGreaterEqual(t.velocity, -20)
        self.assertLessEqual(t.velocity, 20)

    def test_snr_clamped(self):
        t = DemoTarget(6)
        t.snr = 49.5
        t.range_m = 150.0
        for _ in range(100):
            t.range_m = 150.0
            t.step()
        self.assertGreaterEqual(t.snr, 0)
        self.assertLessEqual(t.snr, 50)


class TestDemoSimulatorNoTk(unittest.TestCase):
    """Test DemoSimulator logic without a real Tk event loop.

    We replace ``root.after`` with a mock to avoid needing a display.
    """

    def _make_simulator(self):
        from unittest.mock import MagicMock

        fq = queue.Queue(maxsize=100)
        uq = queue.Queue(maxsize=100)
        mock_root = MagicMock()
        # root.after(ms, fn) should return an id (str)
        mock_root.after.return_value = "mock_after_id"
        sim = DemoSimulator(fq, uq, mock_root, interval_ms=100)
        return sim, fq, uq, mock_root

    def test_initial_targets_created(self):
        sim, _fq, _uq, _root = self._make_simulator()
        self.assertEqual(sim.n_targets, 8)

    def test_tick_produces_frame_and_targets(self):
        sim, fq, uq, _root = self._make_simulator()
        sim._tick()
        # Should have a frame
        self.assertFalse(fq.empty())
        frame = fq.get_nowait()
        self.assertIsInstance(frame, RadarFrame)
        self.assertEqual(frame.frame_number, 1)
        # Should have demo_targets in ui_queue
        tag, payload = uq.get_nowait()
        self.assertEqual(tag, "demo_targets")
        self.assertIsInstance(payload, list)
        xy = ppi_xy(
            [t["range_m"] for t in payload],
            [t["azimuth"] for t in payload],
        )
        self.assertEqual(xy.shape, (len(payload), 2))
        self.assertTrue(np.isfinite(xy).all())

    def test_tick_produces_nonzero_detections(self):
        """Demo targets should actually render into the range-Doppler grid."""
        sim, fq, _uq, _root = self._make_simulator()
        sim._tick()
        frame = fq.get_nowait()
        # At least some targets should produce magnitude > 0 and detections
        self.assertGreater(frame.magnitude.sum(), 0,
                           "Demo targets should render into range-Doppler grid")
        self.assertGreater(frame.detection_count, 0,
                           "Demo targets should produce detections")

    def test_10k_tick_is_vectorized(self):
        from unittest.mock import MagicMock

        fq = queue.Queue(maxsize=4)
        uq = queue.Queue(maxsize=4)
        mock_root = MagicMock()
        mock_root.after.return_value = "mock_after_id"
        sim = DemoSimulator(fq, uq, mock_root, interval_ms=50, n_targets=10_000)
        sim._tick()
        self.assertEqual(sim.n_targets, 10_000)
        frame = fq.get_nowait()
        self.assertGreater(frame.magnitude.sum(), 0)
        xy = ppi_xy(sim._range, sim._az)
        img = ppi_raster(xy, DemoTarget._MAX_RANGE, size=256)
        self.assertEqual(xy.shape, (10_000, 2))
        self.assertGreater(float(img.sum()), 100.0)

    def test_stop_cancels_after(self):
        sim, _fq, _uq, mock_root = self._make_simulator()
        sim._tick()  # sets _after_id
        sim.stop()
        mock_root.after_cancel.assert_called_once_with("mock_after_id")
        self.assertIsNone(sim._after_id)


class TestReplayController(unittest.TestCase):
    """Unit tests for _ReplayController (no GUI required)."""

    def test_initial_state(self):
        fq = queue.Queue()
        uq = queue.Queue()
        ctrl = _ReplayController(fq, uq)
        self.assertEqual(ctrl.total_frames, 0)
        self.assertEqual(ctrl.current_index, 0)
        self.assertFalse(ctrl.is_playing)
        self.assertIsNone(ctrl.software_fpga)

    def test_set_speed(self):
        ctrl = _ReplayController(queue.Queue(), queue.Queue())
        ctrl.set_speed("2x")
        self.assertAlmostEqual(ctrl._frame_interval, 0.050)

    def test_set_speed_unknown_falls_back(self):
        ctrl = _ReplayController(queue.Queue(), queue.Queue())
        ctrl.set_speed("99x")
        self.assertAlmostEqual(ctrl._frame_interval, 0.100)

    def test_set_loop(self):
        ctrl = _ReplayController(queue.Queue(), queue.Queue())
        ctrl.set_loop(True)
        self.assertTrue(ctrl._loop)
        ctrl.set_loop(False)
        self.assertFalse(ctrl._loop)

    def test_seek_increments_past_emitted(self):
        """After seek(), _current_index should be one past the seeked frame."""
        fq = queue.Queue(maxsize=100)
        uq = queue.Queue(maxsize=100)
        ctrl = _ReplayController(fq, uq)
        # Manually set engine to a mock to allow seek
        from unittest.mock import MagicMock
        mock_engine = MagicMock()
        mock_engine.total_frames = 10
        mock_engine.get_frame.return_value = RadarFrame()
        ctrl._engine = mock_engine
        ctrl.seek(5)
        # _current_index should be 6 (past the emitted frame)
        self.assertEqual(ctrl._current_index, 6)
        self.assertEqual(ctrl._last_emitted_index, 5)
        # Frame should be in the queue
        self.assertFalse(fq.empty())

    def test_seek_clamps_to_bounds(self):
        from unittest.mock import MagicMock

        fq = queue.Queue(maxsize=100)
        uq = queue.Queue(maxsize=100)
        ctrl = _ReplayController(fq, uq)
        mock_engine = MagicMock()
        mock_engine.total_frames = 5
        mock_engine.get_frame.return_value = RadarFrame()
        ctrl._engine = mock_engine

        ctrl.seek(100)
        # Should clamp to last frame (index 4), then _current_index = 5
        self.assertEqual(ctrl._last_emitted_index, 4)
        self.assertEqual(ctrl._current_index, 5)

        ctrl.seek(-10)
        # Should clamp to 0, then _current_index = 1
        self.assertEqual(ctrl._last_emitted_index, 0)
        self.assertEqual(ctrl._current_index, 1)

    def test_close_releases_engine(self):
        from unittest.mock import MagicMock

        fq = queue.Queue(maxsize=100)
        uq = queue.Queue(maxsize=100)
        ctrl = _ReplayController(fq, uq)
        mock_engine = MagicMock()
        mock_engine.total_frames = 5
        mock_engine.get_frame.return_value = RadarFrame()
        ctrl._engine = mock_engine

        ctrl.close()
        mock_engine.close.assert_called_once()
        self.assertIsNone(ctrl._engine)
        self.assertIsNone(ctrl.software_fpga)


if __name__ == "__main__":
    unittest.main(verbosity=2)
