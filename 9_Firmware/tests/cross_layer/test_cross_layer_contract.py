"""
Cross-Layer Contract Tests
==========================
Single pytest file orchestrating three tiers of verification:

Tier 1 — Static Contract Parsing:
  Compares Python, Verilog, and C source code at parse-time to catch
  opcode mismatches, bit-width errors, packet constant drift, and
  layout bugs like the status_words[0] 37-bit truncation.

Tier 2 — Verilog Cosimulation (iverilog):
  Compiles and runs tb_cross_layer_ft2232h.v, then parses its output
  files (cmd_results.txt, data_packet.txt, status_packet.txt) and
  runs Python parsers on the captured bytes to verify round-trip
  correctness.

Tier 3 — C Stub Execution:
  Compiles stm32_settings_stub.cpp, generates a binary settings
  packet from Python, runs the stub, and verifies all parsed field
  values match.

The goal is to find UNKNOWN bugs by testing each layer against
independently-derived ground truth — not just checking that two
layers agree (because both could be wrong).
"""

from __future__ import annotations

import os
import struct
import subprocess
import tempfile
from pathlib import Path

import pytest

# Import the contract parsers
import sys

THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(THIS_DIR))
import contract_parser as cp  # noqa: E402

# Also add the GUI dir to import radar_protocol
sys.path.insert(0, str(cp.GUI_DIR))


# ===================================================================
# Helpers
# ===================================================================

IVERILOG = os.environ.get("IVERILOG", "iverilog")
VVP = os.environ.get("VVP", "vvp")
CXX = os.environ.get("CXX", "c++")

# Check tool availability for conditional skipping
_has_iverilog = Path(IVERILOG).exists() if "/" in IVERILOG else bool(
    subprocess.run(["which", IVERILOG], capture_output=True).returncode == 0
)
_has_cxx = subprocess.run(
    [CXX, "--version"], capture_output=True
).returncode == 0


def _parse_hex_results(text: str) -> list[dict[str, str]]:
    """Parse space-separated hex lines from TB output files."""
    rows = []
    for line in text.strip().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        rows.append(line.split())
    return rows


# ===================================================================
# Ground Truth: FPGA register map (independently transcribed)
# ===================================================================
# This is the SINGLE SOURCE OF TRUTH, manually transcribed from
# radar_system_top.v lines 902-945. If any layer disagrees with
# this, it's a bug in that layer.

GROUND_TRUTH_OPCODES = {
    0x01: ("host_radar_mode", 2),
    0x02: ("host_trigger_pulse", 1),   # pulse
    0x03: ("host_detect_threshold", 16),
    0x04: ("host_stream_control", 3),
    0x10: ("host_long_chirp_cycles", 16),
    0x11: ("host_long_listen_cycles", 16),
    0x12: ("host_guard_cycles", 16),
    0x13: ("host_short_chirp_cycles", 16),
    0x14: ("host_short_listen_cycles", 16),
    0x15: ("host_chirps_per_elev", 6),
    0x16: ("host_gain_shift", 4),
    0x20: ("host_range_mode", 2),
    0x21: ("host_cfar_guard", 4),
    0x22: ("host_cfar_train", 5),
    0x23: ("host_cfar_alpha", 8),
    0x24: ("host_cfar_mode", 2),
    0x25: ("host_cfar_enable", 1),
    0x26: ("host_mti_enable", 1),
    0x27: ("host_dc_notch_width", 3),
    0x28: ("host_agc_enable", 1),
    0x29: ("host_agc_target", 8),
    0x2A: ("host_agc_attack", 4),
    0x2B: ("host_agc_decay", 4),
    0x2C: ("host_agc_holdoff", 4),
    0x30: ("host_self_test_trigger", 1),  # pulse
    0x31: ("host_status_request", 1),     # pulse
    0xFF: ("host_status_request", 1),     # alias, pulse
}

GROUND_TRUTH_RESET_DEFAULTS = {
    "host_radar_mode": 1,           # 2'b01
    "host_detect_threshold": 10000,
    "host_stream_control": 7,       # 3'b111
    "host_long_chirp_cycles": 3000,
    "host_long_listen_cycles": 13700,
    "host_guard_cycles": 17540,
    "host_short_chirp_cycles": 50,
    "host_short_listen_cycles": 17450,
    "host_chirps_per_elev": 32,
    "host_gain_shift": 0,
    "host_range_mode": 0,
    "host_cfar_guard": 2,
    "host_cfar_train": 8,
    "host_cfar_alpha": 0x30,
    "host_cfar_mode": 0,
    "host_cfar_enable": 0,
    "host_mti_enable": 0,
    "host_dc_notch_width": 0,
    "host_agc_enable": 0,
    "host_agc_target": 200,
    "host_agc_attack": 1,
    "host_agc_decay": 1,
    "host_agc_holdoff": 4,
}

GROUND_TRUTH_PACKET_CONSTANTS = {
    "data": {"header": 0xAA, "footer": 0x55, "size": 11},
    "status": {"header": 0xBB, "footer": 0x55, "size": 26},
}


# ===================================================================
# TIER 1: Static Contract Parsing
# ===================================================================

class TestTier1OpcodeContract:
    """Verify Python and Verilog opcode sets match ground truth."""

    def test_python_opcodes_match_ground_truth(self):
        """Every Python Opcode must exist in ground truth with correct value."""
        py_opcodes = cp.parse_python_opcodes()
        for val, entry in py_opcodes.items():
            assert val in GROUND_TRUTH_OPCODES, (
                f"Python Opcode {entry.name}=0x{val:02X} not in ground truth! "
                f"Possible phantom opcode (like the 0x06 incident)."
            )

    def test_ground_truth_opcodes_in_python(self):
        """Every ground truth opcode must have a Python enum entry."""
        py_opcodes = cp.parse_python_opcodes()
        for val, (reg, _width) in GROUND_TRUTH_OPCODES.items():
            assert val in py_opcodes, (
                f"Ground truth opcode 0x{val:02X} ({reg}) missing from Python Opcode enum."
            )

    def test_verilog_opcodes_match_ground_truth(self):
        """Every Verilog case entry must exist in ground truth."""
        v_opcodes = cp.parse_verilog_opcodes()
        for val, entry in v_opcodes.items():
            assert val in GROUND_TRUTH_OPCODES, (
                f"Verilog opcode 0x{val:02X} ({entry.register}) not in ground truth."
            )

    def test_ground_truth_opcodes_in_verilog(self):
        """Every ground truth opcode must have a Verilog case entry."""
        v_opcodes = cp.parse_verilog_opcodes()
        for val, (reg, _width) in GROUND_TRUTH_OPCODES.items():
            assert val in v_opcodes, (
                f"Ground truth opcode 0x{val:02X} ({reg}) missing from Verilog case statement."
            )

    def test_python_verilog_bidirectional_match(self):
        """Python and Verilog must have the same set of opcode values."""
        py_set = set(cp.parse_python_opcodes().keys())
        v_set = set(cp.parse_verilog_opcodes().keys())
        py_only = py_set - v_set
        v_only = v_set - py_set
        assert not py_only, f"Opcodes in Python but not Verilog: {[hex(x) for x in py_only]}"
        assert not v_only, f"Opcodes in Verilog but not Python: {[hex(x) for x in v_only]}"

    def test_verilog_register_names_match(self):
        """Verilog case target registers must match ground truth names."""
        v_opcodes = cp.parse_verilog_opcodes()
        for val, (expected_reg, _) in GROUND_TRUTH_OPCODES.items():
            if val in v_opcodes:
                actual_reg = v_opcodes[val].register
                assert actual_reg == expected_reg, (
                    f"Opcode 0x{val:02X}: Verilog writes to '{actual_reg}' "
                    f"but ground truth says '{expected_reg}'"
                )


class TestTier1BitWidths:
    """Verify register widths and opcode bit slices match ground truth."""

    def test_verilog_register_widths(self):
        """Register declarations must match ground truth bit widths."""
        v_widths = cp.parse_verilog_register_widths()
        for reg, expected_width in [
            (name, w) for _, (name, w) in GROUND_TRUTH_OPCODES.items()
        ]:
            if reg in v_widths:
                actual = v_widths[reg]
                assert actual >= expected_width, (
                    f"{reg}: declared {actual}-bit but ground truth says {expected_width}-bit"
                )

    def test_verilog_opcode_bit_slices(self):
        """Opcode case assignments must use correct bit widths from cmd_value."""
        v_opcodes = cp.parse_verilog_opcodes()
        for val, (reg, expected_width) in GROUND_TRUTH_OPCODES.items():
            if val not in v_opcodes:
                continue
            entry = v_opcodes[val]
            if entry.is_pulse:
                continue  # Pulse opcodes don't use cmd_value slicing
            if entry.bit_width > 0:
                assert entry.bit_width >= expected_width, (
                    f"Opcode 0x{val:02X} ({reg}): bit slice {entry.bit_slice} "
                    f"= {entry.bit_width}-bit, expected >= {expected_width}"
                )


class TestTier1StatusWordTruncation:
    """Verify each status_words[] concatenation is exactly 32 bits."""

    def test_status_words_concat_widths_ft2232h(self):
        """Each status_words[] concat must be EXACTLY 32 bits."""
        port_widths = cp.get_usb_interface_port_widths(
            cp.FPGA_DIR / "usb_data_interface_ft2232h.v"
        )
        concats = cp.parse_verilog_status_word_concats(
            cp.FPGA_DIR / "usb_data_interface_ft2232h.v"
        )

        for idx, expr in concats.items():
            result = cp.count_concat_bits(expr, port_widths)
            if result.total_bits < 0:
                pytest.skip(f"status_words[{idx}]: unknown signal width")
            assert result.total_bits == 32, (
                f"status_words[{idx}] is {result.total_bits} bits, not 32! "
                f"{'TRUNCATION' if result.total_bits > 32 else 'UNDERFLOW'} BUG. "
                f"Fragments: {result.fragments}"
            )

    def test_status_words_concat_widths_ft601(self):
        """Same check for the FT601 interface (same bug expected)."""
        ft601_path = cp.FPGA_DIR / "usb_data_interface.v"
        if not ft601_path.exists():
            pytest.skip("FT601 interface file not found")

        port_widths = cp.get_usb_interface_port_widths(ft601_path)
        concats = cp.parse_verilog_status_word_concats(ft601_path)

        for idx, expr in concats.items():
            result = cp.count_concat_bits(expr, port_widths)
            if result.total_bits < 0:
                pytest.skip(f"status_words[{idx}]: unknown signal width")
            assert result.total_bits == 32, (
                f"FT601 status_words[{idx}] is {result.total_bits} bits, not 32! "
                f"{'TRUNCATION' if result.total_bits > 32 else 'UNDERFLOW'} BUG. "
                f"Fragments: {result.fragments}"
            )


class TestTier1StatusFieldPositions:
    """Verify Python status parser bit positions match Verilog layout."""

    def test_python_status_mode_position(self):
        """
        Verify Python reads radar_mode at the correct bit position matching
        the Verilog status_words[0] layout:
        {0xFF[31:24], mode[23:22], stream[21:19], 3'b000[18:16], threshold[15:0]}
        """
        # Get what Python thinks
        py_fields = cp.parse_python_status_fields()
        mode_field = next((f for f in py_fields if f.name == "radar_mode"), None)
        assert mode_field is not None, "radar_mode not found in parse_status_packet"

        # Ground truth: mode is at bits [23:22], so LSB = 22
        expected_shift = 22
        actual_shift = mode_field.lsb

        assert actual_shift == expected_shift, (
            f"Python reads radar_mode at bit {actual_shift} "
            f"but Verilog status_words[0] has mode at bit {expected_shift}."
        )


class TestTier1PacketConstants:
    """Verify packet header/footer/size constants match across layers."""

    def test_python_packet_constants(self):
        """Python constants match ground truth."""
        py = cp.parse_python_packet_constants()
        for ptype, expected in GROUND_TRUTH_PACKET_CONSTANTS.items():
            assert py[ptype].header == expected["header"], (
                f"Python {ptype} header: 0x{py[ptype].header:02X} != 0x{expected['header']:02X}"
            )
            assert py[ptype].footer == expected["footer"], (
                f"Python {ptype} footer: 0x{py[ptype].footer:02X} != 0x{expected['footer']:02X}"
            )
            assert py[ptype].size == expected["size"], (
                f"Python {ptype} size: {py[ptype].size} != {expected['size']}"
            )

    def test_verilog_packet_constants(self):
        """Verilog localparams match ground truth."""
        v = cp.parse_verilog_packet_constants()
        for ptype, expected in GROUND_TRUTH_PACKET_CONSTANTS.items():
            assert v[ptype].header == expected["header"], (
                f"Verilog {ptype} header: 0x{v[ptype].header:02X} != 0x{expected['header']:02X}"
            )
            assert v[ptype].footer == expected["footer"], (
                f"Verilog {ptype} footer: 0x{v[ptype].footer:02X} != 0x{expected['footer']:02X}"
            )
            assert v[ptype].size == expected["size"], (
                f"Verilog {ptype} size: {v[ptype].size} != {expected['size']}"
            )

    def test_python_verilog_constants_agree(self):
        """Python and Verilog packet constants must match each other."""
        py = cp.parse_python_packet_constants()
        v = cp.parse_verilog_packet_constants()
        for ptype in ("data", "status"):
            assert py[ptype].header == v[ptype].header
            assert py[ptype].footer == v[ptype].footer
            assert py[ptype].size == v[ptype].size


class TestTier1ResetDefaults:
    """Verify Verilog reset defaults match ground truth."""

    def test_verilog_reset_defaults(self):
        """Reset block values must match ground truth."""
        v_defaults = cp.parse_verilog_reset_defaults()
        for reg, expected in GROUND_TRUTH_RESET_DEFAULTS.items():
            assert reg in v_defaults, f"{reg} not found in reset block"
            actual = v_defaults[reg]
            assert actual == expected, (
                f"{reg}: reset default {actual} != expected {expected}"
            )


class TestTier1DataPacketLayout:
    """Verify data packet byte layout matches between Python and Verilog."""

    def test_verilog_data_mux_field_positions(self):
        """Verilog data_pkt_byte mux must have correct byte positions."""
        v_fields = cp.parse_verilog_data_mux()
        # Expected: range_profile at bytes 1-4 (32-bit), doppler_real 5-6,
        #           doppler_imag 7-8, cfar 9
        field_map = {f.name: f for f in v_fields}

        assert "range_profile" in field_map
        rp = field_map["range_profile"]
        assert rp.byte_start == 1 and rp.byte_end == 4 and rp.width_bits == 32

        assert "doppler_real" in field_map
        dr = field_map["doppler_real"]
        assert dr.byte_start == 5 and dr.byte_end == 6 and dr.width_bits == 16

        assert "doppler_imag" in field_map
        di = field_map["doppler_imag"]
        assert di.byte_start == 7 and di.byte_end == 8 and di.width_bits == 16

    def test_python_data_packet_byte_positions(self):
        """Python parse_data_packet byte offsets must be correct."""
        py_fields = cp.parse_python_data_packet_fields()
        # range_q at offset 1 (2B), range_i at offset 3 (2B),
        # doppler_i at offset 5 (2B), doppler_q at offset 7 (2B),
        # detection at offset 9
        field_map = {f.name: f for f in py_fields}

        assert "range_q" in field_map
        assert field_map["range_q"].byte_start == 1
        assert "range_i" in field_map
        assert field_map["range_i"].byte_start == 3
        assert "doppler_i" in field_map
        assert field_map["doppler_i"].byte_start == 5
        assert "doppler_q" in field_map
        assert field_map["doppler_q"].byte_start == 7
        assert "detection" in field_map
        assert field_map["detection"].byte_start == 9


class TestTier1STM32SettingsPacket:
    """Verify STM32 settings packet layout."""

    def test_field_order_and_sizes(self):
        """STM32 settings fields must have correct offsets and sizes."""
        fields = cp.parse_stm32_settings_fields()
        if not fields:
            pytest.skip("MCU source not available")

        expected = [
            ("system_frequency", 0, 8, "double"),
            ("chirp_duration_1", 8, 8, "double"),
            ("chirp_duration_2", 16, 8, "double"),
            ("chirps_per_position", 24, 4, "uint32_t"),
            ("freq_min", 28, 8, "double"),
            ("freq_max", 36, 8, "double"),
            ("prf1", 44, 8, "double"),
            ("prf2", 52, 8, "double"),
            ("max_distance", 60, 8, "double"),
            ("map_size", 68, 8, "double"),
        ]

        assert len(fields) == len(expected), (
            f"Expected {len(expected)} fields, got {len(fields)}"
        )

        for f, (ename, eoff, esize, etype) in zip(fields, expected, strict=True):
            assert f.name == ename, f"Field name: {f.name} != {ename}"
            assert f.offset == eoff, f"{f.name}: offset {f.offset} != {eoff}"
            assert f.size == esize, f"{f.name}: size {f.size} != {esize}"
            assert f.c_type == etype, f"{f.name}: type {f.c_type} != {etype}"

    def test_minimum_packet_size(self):
        """
        RadarSettings.cpp says minimum is 74 bytes but actual payload is:
        'SET'(3) + 9*8(doubles) + 4(uint32) + 'END'(3) = 82 bytes.
        This test documents the bug.
        """
        fields = cp.parse_stm32_settings_fields()
        if not fields:
            pytest.skip("MCU source not available")

        # Calculate required payload size
        total_field_bytes = sum(f.size for f in fields)
        # Add markers: "SET"(3) + "END"(3)
        required_size = 3 + total_field_bytes + 3

        # Read the actual minimum check from the source
        src = (cp.MCU_LIB_DIR / "RadarSettings.cpp").read_text(encoding="latin-1")
        import re
        m = re.search(r'length\s*<\s*(\d+)', src)
        assert m, "Could not find minimum length check in parseFromUSB"
        declared_min = int(m.group(1))

        assert declared_min == required_size, (
            f"BUFFER OVERREAD BUG: parseFromUSB minimum check is {declared_min} "
            f"but actual required size is {required_size}. "
            f"({total_field_bytes} bytes of fields + 6 bytes of markers). "
            f"If exactly {declared_min} bytes are passed, extractDouble() reads "
            f"past the buffer at offset {declared_min - 3} (needs 8 bytes, "
            f"only {declared_min - 3 - fields[-1].offset} available)."
        )

    def test_stm32_usb_start_flag(self):
        """USB start flag must be [23, 46, 158, 237]."""
        flag = cp.parse_stm32_start_flag()
        if not flag:
            pytest.skip("USBHandler.cpp not available")
        assert flag == [23, 46, 158, 237], f"Start flag: {flag}"


# ===================================================================
# TIER 2: Verilog Cosimulation
# ===================================================================

@pytest.mark.skipif(not _has_iverilog, reason="iverilog not available")
class TestTier2VerilogCosim:
    """Compile and run the FT2232H TB, validate output against Python parsers."""

    @pytest.fixture(scope="class")
    def tb_results(self, tmp_path_factory):
        """Compile and run TB once, return output file contents."""
        workdir = tmp_path_factory.mktemp("verilog_cosim")

        tb_path = THIS_DIR / "tb_cross_layer_ft2232h.v"
        rtl_path = cp.FPGA_DIR / "usb_data_interface_ft2232h.v"
        out_bin = workdir / "tb_cross_layer_ft2232h"

        # Compile
        result = subprocess.run(
            [IVERILOG, "-o", str(out_bin), "-I", str(cp.FPGA_DIR),
             str(tb_path), str(rtl_path)],
            capture_output=True, text=True, timeout=30,
        )
        assert result.returncode == 0, f"iverilog compile failed:\n{result.stderr}"

        # Run
        result = subprocess.run(
            [VVP, str(out_bin)],
            capture_output=True, text=True, timeout=60,
            cwd=str(workdir),
        )
        assert result.returncode == 0, f"vvp failed:\n{result.stderr}"

        # Parse output
        return {
            "stdout": result.stdout,
            "cmd_results": (workdir / "cmd_results.txt").read_text(),
            "data_packet": (workdir / "data_packet.txt").read_text(),
            "status_packet": (workdir / "status_packet.txt").read_text(),
        }

    def test_all_tb_tests_pass(self, tb_results):
        """All Verilog TB internal checks must pass."""
        stdout = tb_results["stdout"]
        assert "ALL TESTS PASSED" in stdout, f"TB had failures:\n{stdout}"

    def test_command_round_trip(self, tb_results):
        """Verify every command decoded correctly by matching sent vs received."""
        rows = _parse_hex_results(tb_results["cmd_results"])
        assert len(rows) >= 20, f"Expected >= 20 command results, got {len(rows)}"

        for row in rows:
            assert len(row) == 6, f"Bad row format: {row}"
            sent_op, sent_addr, sent_val = row[0], row[1], row[2]
            got_op, got_addr, got_val = row[3], row[4], row[5]
            assert sent_op == got_op, (
                f"Opcode mismatch: sent 0x{sent_op} got 0x{got_op}"
            )
            assert sent_addr == got_addr, (
                f"Addr mismatch: sent 0x{sent_addr} got 0x{got_addr}"
            )
            assert sent_val == got_val, (
                f"Value mismatch: sent 0x{sent_val} got 0x{got_val}"
            )

    def test_data_packet_python_round_trip(self, tb_results):
        """
        Take the 11 bytes captured by the Verilog TB, run Python's
        parse_data_packet() on them, verify the parsed values match
        what was injected into the TB.
        """
        from radar_protocol import RadarProtocol

        rows = _parse_hex_results(tb_results["data_packet"])
        assert len(rows) == 11, f"Expected 11 data packet bytes, got {len(rows)}"

        # Reconstruct raw bytes
        raw = bytes(int(row[1], 16) for row in rows)
        assert len(raw) == 11

        parsed = RadarProtocol.parse_data_packet(raw)
        assert parsed is not None, "parse_data_packet returned None"

        # The TB injected: range_profile = 0xCAFE_BEEF = {Q=0xCAFE, I=0xBEEF}
        #   doppler_real = 0x1234, doppler_imag = 0x5678
        #   cfar_detection = 1
        #
        # range_q = 0xCAFE → signed = 0xCAFE - 0x10000 = -13570
        # range_i = 0xBEEF → signed = 0xBEEF - 0x10000 = -16657
        # doppler_i = 0x1234 → signed = 4660
        # doppler_q = 0x5678 → signed = 22136

        assert parsed["range_q"] == (0xCAFE - 0x10000), (
            f"range_q: {parsed['range_q']} != {0xCAFE - 0x10000}"
        )
        assert parsed["range_i"] == (0xBEEF - 0x10000), (
            f"range_i: {parsed['range_i']} != {0xBEEF - 0x10000}"
        )
        assert parsed["doppler_i"] == 0x1234, (
            f"doppler_i: {parsed['doppler_i']} != {0x1234}"
        )
        assert parsed["doppler_q"] == 0x5678, (
            f"doppler_q: {parsed['doppler_q']} != {0x5678}"
        )
        assert parsed["detection"] == 1, (
            f"detection: {parsed['detection']} != 1"
        )

    def test_status_packet_python_round_trip(self, tb_results):
        """
        Take the 26 bytes captured by the Verilog TB, run Python's
        parse_status_packet() on them, verify against injected values.
        """
        from radar_protocol import RadarProtocol

        lines = tb_results["status_packet"].strip().splitlines()
        # Filter out comments and status_words debug lines
        rows = []
        for line in lines:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            rows.append(line.split())

        assert len(rows) == 26, f"Expected 26 status bytes, got {len(rows)}"

        raw = bytes(int(row[1], 16) for row in rows)
        assert len(raw) == 26

        sr = RadarProtocol.parse_status_packet(raw)
        assert sr is not None, "parse_status_packet returned None"

        # Injected values (from TB):
        #   status_cfar_threshold = 0xABCD
        #   status_stream_ctrl = 3'b101 = 5
        #   status_radar_mode = 2'b11 = 3
        #   status_long_chirp = 0x1234
        #   status_long_listen = 0x5678
        #   status_guard = 0x9ABC
        #   status_short_chirp = 0xDEF0
        #   status_short_listen = 0xFACE
        #   status_chirps_per_elev = 42
        #   status_range_mode = 2'b10 = 2
        #   status_self_test_flags = 5'b10101 = 21
        #   status_self_test_detail = 0xA5
        #   status_self_test_busy = 1
        #   status_agc_current_gain = 7
        #   status_agc_peak_magnitude = 200
        #   status_agc_saturation_count = 15
        #   status_agc_enable = 1

        # Words 1-5 should be correct (no truncation bug)
        assert sr.cfar_threshold == 0xABCD, f"cfar_threshold: 0x{sr.cfar_threshold:04X}"
        assert sr.long_chirp == 0x1234, f"long_chirp: 0x{sr.long_chirp:04X}"
        assert sr.long_listen == 0x5678, f"long_listen: 0x{sr.long_listen:04X}"
        assert sr.guard == 0x9ABC, f"guard: 0x{sr.guard:04X}"
        assert sr.short_chirp == 0xDEF0, f"short_chirp: 0x{sr.short_chirp:04X}"
        assert sr.short_listen == 0xFACE, f"short_listen: 0x{sr.short_listen:04X}"
        assert sr.chirps_per_elev == 42, f"chirps_per_elev: {sr.chirps_per_elev}"
        assert sr.range_mode == 2, f"range_mode: {sr.range_mode}"
        assert sr.self_test_flags == 21, f"self_test_flags: {sr.self_test_flags}"
        assert sr.self_test_detail == 0xA5, f"self_test_detail: 0x{sr.self_test_detail:02X}"
        assert sr.self_test_busy == 1, f"self_test_busy: {sr.self_test_busy}"

        # AGC fields (word 4)
        assert sr.agc_current_gain == 7, f"agc_current_gain: {sr.agc_current_gain}"
        assert sr.agc_peak_magnitude == 200, f"agc_peak_magnitude: {sr.agc_peak_magnitude}"
        assert sr.agc_saturation_count == 15, f"agc_saturation_count: {sr.agc_saturation_count}"
        assert sr.agc_enable == 1, f"agc_enable: {sr.agc_enable}"

        # Word 0: stream_ctrl should be 5 (3'b101)
        assert sr.stream_ctrl == 5, (
            f"stream_ctrl: {sr.stream_ctrl} != 5. "
            f"Check status_words[0] bit positions."
        )

        # radar_mode should be 3 (2'b11)
        assert sr.radar_mode == 3, (
            f"radar_mode={sr.radar_mode} != 3. "
            f"Check status_words[0] bit positions."
        )


# ===================================================================
# TIER 3: C Stub Execution
# ===================================================================

@pytest.mark.skipif(not _has_cxx, reason="C++ compiler not available")
class TestTier3CStub:
    """Compile STM32 settings stub and verify field parsing."""

    @pytest.fixture(scope="class")
    def stub_binary(self, tmp_path_factory):
        """Compile the C++ stub once."""
        workdir = tmp_path_factory.mktemp("c_stub")
        stub_src = THIS_DIR / "stm32_settings_stub.cpp"
        radar_settings_src = cp.MCU_LIB_DIR / "RadarSettings.cpp"
        out_bin = workdir / "stm32_settings_stub"

        result = subprocess.run(
            [CXX, "-std=c++11", "-o", str(out_bin),
             str(stub_src), str(radar_settings_src),
             f"-I{cp.MCU_LIB_DIR}"],
            capture_output=True, text=True, timeout=30,
        )
        assert result.returncode == 0, f"Compile failed:\n{result.stderr}"
        return out_bin

    def _build_settings_packet(self, values: dict) -> bytes:
        """Build a binary settings packet matching RadarSettings::parseFromUSB."""
        pkt = b"SET"
        for key in [
            "system_frequency", "chirp_duration_1", "chirp_duration_2",
        ]:
            pkt += struct.pack(">d", values[key])
        pkt += struct.pack(">I", values["chirps_per_position"])
        for key in [
            "freq_min", "freq_max", "prf1", "prf2",
            "max_distance", "map_size",
        ]:
            pkt += struct.pack(">d", values[key])
        pkt += b"END"
        return pkt

    def _run_stub(self, binary: Path, packet: bytes) -> dict[str, str]:
        """Run stub with packet file, parse stdout into field dict."""
        with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as f:
            f.write(packet)
            pkt_path = f.name

        try:
            result = subprocess.run(
                [str(binary), pkt_path],
                capture_output=True, text=True, timeout=10,
            )
        finally:
            os.unlink(pkt_path)

        fields = {}
        for line in result.stdout.strip().splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                fields[k.strip()] = v.strip()
        return fields

    def test_default_values_round_trip(self, stub_binary):
        """Default settings must parse correctly through C stub."""
        values = {
            "system_frequency": 10.0e9,
            "chirp_duration_1": 30.0e-6,
            "chirp_duration_2": 0.5e-6,
            "chirps_per_position": 32,
            "freq_min": 10.0e6,
            "freq_max": 30.0e6,
            "prf1": 1000.0,
            "prf2": 2000.0,
            "max_distance": 50000.0,
            "map_size": 50000.0,
        }
        pkt = self._build_settings_packet(values)
        result = self._run_stub(stub_binary, pkt)

        assert result.get("parse_ok") == "true", f"Parse failed: {result}"

        for key, expected in values.items():
            actual_str = result.get(key)
            assert actual_str is not None, f"Missing field: {key}"
            actual = int(actual_str) if key == "chirps_per_position" else float(actual_str)
            if isinstance(expected, float):
                assert abs(actual - expected) < expected * 1e-10, (
                    f"{key}: {actual} != {expected}"
                )
            else:
                assert actual == expected, f"{key}: {actual} != {expected}"

    def test_distinctive_values_round_trip(self, stub_binary):
        """Non-default distinctive values must parse correctly."""
        values = {
            "system_frequency": 24.125e9,   # K-band
            "chirp_duration_1": 100.0e-6,
            "chirp_duration_2": 2.0e-6,
            "chirps_per_position": 64,
            "freq_min": 24.0e6,
            "freq_max": 24.25e6,
            "prf1": 5000.0,
            "prf2": 3000.0,
            "max_distance": 75000.0,
            "map_size": 100000.0,
        }
        pkt = self._build_settings_packet(values)
        result = self._run_stub(stub_binary, pkt)

        assert result.get("parse_ok") == "true", f"Parse failed: {result}"

        for key, expected in values.items():
            actual_str = result.get(key)
            assert actual_str is not None, f"Missing field: {key}"
            actual = int(actual_str) if key == "chirps_per_position" else float(actual_str)
            if isinstance(expected, float):
                assert abs(actual - expected) < expected * 1e-10, (
                    f"{key}: {actual} != {expected}"
                )
            else:
                assert actual == expected, f"{key}: {actual} != {expected}"

    def test_truncated_packet_rejected(self, stub_binary):
        """Packet shorter than minimum must be rejected."""
        pkt = b"SET" + b"\x00" * 40 + b"END"  # Only 46 bytes, needs 82
        result = self._run_stub(stub_binary, pkt)
        assert result.get("parse_ok") == "false", (
            f"Expected parse failure for truncated packet, got: {result}"
        )

    def test_bad_markers_rejected(self, stub_binary):
        """Packet with wrong start/end markers must be rejected."""
        values = {
            "system_frequency": 10.0e9, "chirp_duration_1": 30.0e-6,
            "chirp_duration_2": 0.5e-6, "chirps_per_position": 32,
            "freq_min": 10.0e6, "freq_max": 30.0e6,
            "prf1": 1000.0, "prf2": 2000.0,
            "max_distance": 50000.0, "map_size": 50000.0,
        }
        pkt = self._build_settings_packet(values)

        # Wrong start marker
        bad_pkt = b"BAD" + pkt[3:]
        result = self._run_stub(stub_binary, bad_pkt)
        assert result.get("parse_ok") == "false", "Should reject bad start marker"

        # Wrong end marker
        bad_pkt = pkt[:-3] + b"BAD"
        result = self._run_stub(stub_binary, bad_pkt)
        assert result.get("parse_ok") == "false", "Should reject bad end marker"

    def test_python_c_packet_format_agreement(self, stub_binary):
        """
        Python builds a settings packet, C stub parses it.
        This tests that both sides agree on the packet format.
        """
        # Use values right at validation boundaries to stress-test
        values = {
            "system_frequency": 1.0e9,     # min valid
            "chirp_duration_1": 1.0e-6,    # min valid
            "chirp_duration_2": 0.1e-6,    # min valid
            "chirps_per_position": 1,      # min valid
            "freq_min": 1.0e6,             # min valid
            "freq_max": 2.0e6,             # just above freq_min
            "prf1": 100.0,                 # min valid
            "prf2": 100.0,                 # min valid
            "max_distance": 100.0,         # min valid
            "map_size": 1000.0,            # min valid
        }
        pkt = self._build_settings_packet(values)
        result = self._run_stub(stub_binary, pkt)

        assert result.get("parse_ok") == "true", (
            f"Boundary values rejected: {result}"
        )
