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
import re
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

# In CI, missing tools must be a hard failure — never silently skip.
_in_ci = os.environ.get("GITHUB_ACTIONS") == "true"
if _in_ci:
    if not _has_iverilog:
        raise RuntimeError(
            "iverilog is required in CI but was not found. "
            "Ensure 'apt-get install iverilog' ran and IVERILOG/VVP are on PATH."
        )
    if not _has_cxx:
        raise RuntimeError(
            "C++ compiler is required in CI but was not found. "
            "Ensure build-essential is installed."
        )


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


class TestTier1AgcCrossLayerInvariant:
    """
    Verify AGC enable/disable is consistent across FPGA, MCU, and GUI layers.

    System-level invariant: the FPGA register host_agc_enable is the single
    source of truth for AGC state. It propagates to MCU via DIG_6 GPIO and
    to GUI via status word 4 bit[11]. At boot, all layers must agree AGC=OFF.
    At runtime, the MCU must read DIG_6 every frame to sync its outer-loop AGC.
    """

    def test_fpga_dig6_drives_agc_enable(self):
        """FPGA must drive gpio_dig6 from host_agc_enable, NOT tied low."""
        rtl = (cp.FPGA_DIR / "radar_system_top.v").read_text()
        # Must find: assign gpio_dig6 = host_agc_enable;
        assert re.search(
            r'assign\s+gpio_dig6\s*=\s*host_agc_enable\s*;', rtl
        ), "gpio_dig6 must be driven by host_agc_enable (not tied low)"
        # Must NOT have the old tied-low pattern
        assert not re.search(
            r"assign\s+gpio_dig6\s*=\s*1'b0\s*;", rtl
        ), "gpio_dig6 must NOT be tied low — it carries AGC enable"

    def test_fpga_agc_enable_boot_default_off(self):
        """FPGA host_agc_enable must reset to 0 (AGC off at boot)."""
        v_defaults = cp.parse_verilog_reset_defaults()
        assert "host_agc_enable" in v_defaults, (
            "host_agc_enable not found in reset block"
        )
        assert v_defaults["host_agc_enable"] == 0, (
            f"host_agc_enable reset default is {v_defaults['host_agc_enable']}, "
            "expected 0 (AGC off at boot)"
        )

    def test_mcu_agc_constructor_default_off(self):
        """MCU ADAR1000_AGC constructor must default enabled=false."""
        agc_cpp = (cp.MCU_LIB_DIR / "ADAR1000_AGC.cpp").read_text()
        # The constructor initializer list must have enabled(false)
        assert re.search(
            r'enabled\s*\(\s*false\s*\)', agc_cpp
        ), "ADAR1000_AGC constructor must initialize enabled(false)"
        assert not re.search(
            r'enabled\s*\(\s*true\s*\)', agc_cpp
        ), "ADAR1000_AGC constructor must NOT initialize enabled(true)"

    def test_mcu_reads_dig6_before_agc_gate(self):
        """MCU main loop must read DIG_6 GPIO to sync outerAgc.enabled."""
        main_cpp = (cp.MCU_CODE_DIR / "main.cpp").read_text()
        # DIG_6 must be read via HAL_GPIO_ReadPin
        assert re.search(
            r'HAL_GPIO_ReadPin\s*\(\s*FPGA_DIG6', main_cpp,
        ), "main.cpp must read DIG_6 GPIO via HAL_GPIO_ReadPin"
        # outerAgc.enabled must be assigned from the DIG_6 reading
        # (may be indirect via debounce variable like dig6_now)
        assert re.search(
            r'outerAgc\.enabled\s*=', main_cpp,
        ), "main.cpp must assign outerAgc.enabled from DIG_6 state"

    def test_boot_invariant_all_layers_agc_off(self):
        """
        At boot, all three layers must agree: AGC is OFF.
        - FPGA: host_agc_enable resets to 0 -> DIG_6 low
        - MCU: ADAR1000_AGC.enabled defaults to false
        - GUI: reads status word 4 bit[11] = 0 -> reports MANUAL
        """
        # FPGA
        v_defaults = cp.parse_verilog_reset_defaults()
        assert v_defaults.get("host_agc_enable") == 0

        # MCU
        agc_cpp = (cp.MCU_LIB_DIR / "ADAR1000_AGC.cpp").read_text()
        assert re.search(r'enabled\s*\(\s*false\s*\)', agc_cpp)

        # GUI: status word 4 bit[11] is host_agc_enable, which resets to 0.
        # Verify the GUI parses bit[11] of status word 4 as the AGC flag.
        gui_py = (cp.GUI_DIR / "radar_protocol.py").read_text()
        assert re.search(
            r'words\[4\].*>>\s*11|status_words\[4\].*>>\s*11',
            gui_py,
        ), "GUI must parse AGC status from words[4] bit[11]"

    def test_status_word4_agc_bit_matches_dig6_source(self):
        """
        Status word 4 bit[11] and DIG_6 must both derive from host_agc_enable.
        This guarantees the GUI status display can never lie about MCU AGC state.
        """
        rtl = (cp.FPGA_DIR / "radar_system_top.v").read_text()

        # DIG_6 driven by host_agc_enable
        assert re.search(
            r'assign\s+gpio_dig6\s*=\s*host_agc_enable\s*;', rtl
        )

        # Status word 4 must contain host_agc_enable (may be named
        # status_agc_enable at the USB interface port boundary).
        # Also verify the top-level wiring connects them.
        usb_ft2232h = (cp.FPGA_DIR / "usb_data_interface_ft2232h.v").read_text()
        usb_ft601 = (cp.FPGA_DIR / "usb_data_interface.v").read_text()

        # USB interfaces use the port name status_agc_enable
        found_in_ft2232h = "status_agc_enable" in usb_ft2232h
        found_in_ft601 = "status_agc_enable" in usb_ft601

        assert found_in_ft2232h or found_in_ft601, (
            "status_agc_enable must appear in at least one USB interface's "
            "status word to guarantee GUI status matches DIG_6"
        )

        # Verify top-level wiring: status_agc_enable port is connected
        # to host_agc_enable (same signal that drives DIG_6)
        assert re.search(
            r'\.status_agc_enable\s*\(\s*host_agc_enable\s*\)', rtl
        ), (
            "Top-level must wire .status_agc_enable(host_agc_enable) "
            "so status word and DIG_6 derive from the same signal"
        )

    def test_mcu_dig6_debounce_guards_enable_assignment(self):
        """
        MCU must apply a 2-frame confirmation debounce before mutating
        outerAgc.enabled from DIG_6 reads. A naive assignment straight from
        the latest GPIO sample would let a single-cycle glitch flip the AGC
        state for one frame — defeating the debounce claim in the PR body.
        """
        main_cpp = (cp.MCU_CODE_DIR / "main.cpp").read_text()

        # (1) Current-frame DIG_6 sample must be captured in a local variable
        # so it can be compared against the previous-frame value.
        now_match = re.search(
            r'(bool|int|uint8_t)\s+(\w*dig6\w*)\s*=\s*[^;]*?'
            r'HAL_GPIO_ReadPin\s*\(\s*FPGA_DIG6[^;]*;',
            main_cpp,
            re.DOTALL,
        )
        assert now_match, (
            "DIG_6 read must be stored in a local variable (e.g. `dig6_now`) "
            "so the current sample can be compared against the previous frame"
        )
        now_var = now_match.group(2)

        # (2) Previous-frame state must persist across iterations via static
        # storage, and must default to false (matches FPGA boot: AGC off).
        prev_match = re.search(
            r'static\s+(bool|int|uint8_t)\s+(\w*dig6\w*)\s*=\s*(false|0)\s*;',
            main_cpp,
        )
        assert prev_match, (
            "A static previous-frame variable (e.g. "
            "`static bool dig6_prev = false;`) must exist, initialized to "
            "false so the debounce starts in sync with the FPGA boot default"
        )
        prev_var = prev_match.group(2)
        assert prev_var != now_var, (
            f"Current and previous DIG_6 variables must be distinct "
            f"(both are '{now_var}')"
        )

        # (3) outerAgc.enabled assignment must be gated by now == prev.
        guarded_assign = re.search(
            rf'if\s*\(\s*{now_var}\s*==\s*{prev_var}\s*\)\s*\{{[^}}]*?'
            rf'outerAgc\.enabled\s*=\s*{now_var}\s*;',
            main_cpp,
            re.DOTALL,
        )
        assert guarded_assign, (
            f"`outerAgc.enabled = {now_var};` must be inside "
            f"`if ({now_var} == {prev_var}) {{ ... }}` — the confirmation "
            "guard that absorbs single-sample GPIO glitches. A naive "
            "assignment without this guard reintroduces the glitch bug."
        )

        # (4) Previous-frame variable must advance each frame.
        prev_update = re.search(
            rf'{prev_var}\s*=\s*{now_var}\s*;',
            main_cpp,
        )
        assert prev_update, (
            f"`{prev_var} = {now_var};` must run each frame so the "
            "debounce window slides forward; without it the guard is "
            "stuck and enable changes never confirm"
        )


class TestTier1Adar1000ChannelConvention:
    """
    Verify ADAR1000Manager channel-indexed register helpers agree with
    their callers' convention.

    Callers (ADAR1000_AGC.cpp and ADAR1000_Manager.cpp internal call
    sites) pass 1-based channel: `mgr.adarSetRxVgaGain(dev, ch + 1, ...)`
    where `ch` iterates 0..3. The comment on ADAR1000_AGC.cpp explicitly
    states "Channel parameter is 1-based per Manager convention".

    The helpers, however, compute the register address as
    `REG_CH1_* + (channel & 0x03) * stride`. With 1-based input this
    produces:

        caller ch=0 (hw CH1) -> channel=1 -> offset 1 -> REG_CH2
        caller ch=1 (hw CH2) -> channel=2 -> offset 2 -> REG_CH3
        caller ch=2 (hw CH3) -> channel=3 -> offset 3 -> REG_CH4
        caller ch=3 (hw CH4) -> channel=4 -> offset 0 -> REG_CH1

    i.e. every write is rotated +1 mod 4. See upstream issue #90.

    The fix collapses one side:
        (a) callers switch to 0-based `ch`; helpers keep `(channel & 0x03)`
        (b) callers keep `ch + 1`; helpers use `(channel - 1)`

    Either fix drops one of the two counted populations to zero.
    """

    _ADAR_CHANNEL_SETTERS = (
        "adarSetRxPhase",
        "adarSetTxPhase",
        "adarSetRxVgaGain",
        "adarSetTxVgaGain",
    )

    @pytest.mark.xfail(
        strict=True,
        reason=(
            "#90 — ADAR1000Manager channel helpers use `(channel & 0x03)` "
            "while all call sites pass 1-based `ch + 1`, rotating writes by "
            "+1 mod 4. Upstream issue open as of 2026-04-17."
        ),
    )
    def test_adar1000_caller_helper_channel_convention_agrees(self):
        """Caller indexing base must match helper indexing base."""
        manager_cpp = (cp.MCU_LIB_DIR / "ADAR1000_Manager.cpp").read_text()
        agc_cpp = (cp.MCU_LIB_DIR / "ADAR1000_AGC.cpp").read_text()

        # Count call sites passing 1-based `ch + 1` (or `(ch + 1)`) as the
        # channel argument to any ADAR1000 channel-indexed setter.
        one_based_call_sites = 0
        for fn in self._ADAR_CHANNEL_SETTERS:
            pattern = (
                rf'{fn}\s*\([^)]*?,\s*\(?\s*ch\s*\+\s*1\s*\)?\s*,'
            )
            for text in (manager_cpp, agc_cpp):
                one_based_call_sites += len(re.findall(pattern, text))

        # Count helper sites that index REG_CH1_* via `(channel & 0x03)`.
        masked_helper_sites = len(
            re.findall(
                r'REG_CH1_\w+\s*\+\s*\(\s*channel\s*&\s*0x03\s*\)',
                manager_cpp,
            )
        )

        # If both populations are non-empty, the conventions disagree and
        # every channel write is rotated +1 mod 4.
        assert not (one_based_call_sites > 0 and masked_helper_sites > 0), (
            f"ADAR1000 channel convention mismatch: "
            f"{one_based_call_sites} call sites pass 1-based `ch + 1` while "
            f"{masked_helper_sites} helper sites compute "
            f"`REG_CH1_* + (channel & 0x03)` (0-based). "
            "See upstream issue #90 — rotates CH1..CH4 writes by +1 mod 4."
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
