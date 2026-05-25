#!/bin/bash
# ===========================================================================
# FPGA Regression Test Runner for AERIS-10 Radar
# Phase 0: Vivado-style lint (catches issues iverilog silently accepts)
# Phase 1+: Compile and run all verified iverilog testbenches
#
# Usage:  ./run_regression.sh [--quick] [--skip-lint]
#   --quick      Skip long-running integration tests (receiver golden, system TB)
#   --skip-lint  Skip Phase 0 lint checks (not recommended)
#
# Exit code: 0 if all tests pass, 1 if any fail
# ===========================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

QUICK=0
SKIP_LINT=0
for arg in "$@"; do
    case "$arg" in
        --quick) QUICK=1 ;;
        --skip-lint) SKIP_LINT=1 ;;
    esac
done

# GNU timeout is not present on macOS by default. Prefer `gtimeout` from
# `brew install coreutils` if available, otherwise fail fast with a clear
# hint — silent fall-through previously made every vvp run look like
# "UNKNOWN (no PASS/FAIL markers)".
if command -v timeout >/dev/null 2>&1; then
    TIMEOUT_CMD=timeout
elif command -v gtimeout >/dev/null 2>&1; then
    TIMEOUT_CMD=gtimeout
else
    echo "ERROR: GNU timeout not found. On macOS: brew install coreutils" >&2
    exit 1
fi

PASS=0
FAIL=0
SKIP=0
LINT_WARN=0
LINT_ERR=0
ERRORS=""

# Colors (if terminal supports it)
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# ===========================================================================
# PHASE 0: VIVADO-STYLE LINT
# Two layers:
#   (A) iverilog -Wall full-design compile — parse for serious warnings
#   (B) Custom regex checks for patterns Vivado treats as errors
# ===========================================================================

# Production RTL file list (same as system TB minus testbench files)
# Uses ADC stub for IBUFDS/BUFIO primitives that iverilog can't parse
PROD_RTL=(
    radar_system_top.v
    radar_transmitter.v
    dac_interface_single.v
    plfm_chirp_controller.v
    radar_receiver_final.v
    tb/ad9484_interface_400m_stub.v
    ddc_400m.v
    nco_400m_enhanced.v
    cic_decimator_4x_enhanced.v
    cdc_modules.v
    fir_lowpass.v
    ddc_input_interface.v
    chirp_memory_loader_param.v
    latency_buffer.v
    matched_filter_multi_segment.v
    matched_filter_processing_chain.v
    range_bin_decimator.v
    doppler_processor.v
    xfft_16.v
    fft_engine.v
    usb_data_interface.v
    usb_data_interface_ft2232h.v
    edge_detector.v
    radar_mode_controller.v
    rx_gain_control.v
    cfar_ca.v
    mti_canceller.v
    fpga_self_test.v
)

# Source-only RTL (not instantiated at top level, but should still be lint-clean)
# Note: ad9484_interface_400m.v is excluded — it uses Xilinx primitives
# (IBUFDS, BUFIO, BUFG, IDDR) that iverilog cannot compile. The production
# design uses tb/ad9484_interface_400m_stub.v for simulation instead.
EXTRA_RTL=(
    frequency_matched_filter.v
)

# ---------------------------------------------------------------------------
# Shared RTL file lists for integration / system tests
# Centralised here so a new module only needs adding once.
# ---------------------------------------------------------------------------

# Receiver chain (used by golden generate/compare tests)
RECEIVER_RTL=(
    radar_receiver_final.v
    radar_mode_controller.v
    tb/ad9484_interface_400m_stub.v
    ddc_400m.v nco_400m_enhanced.v cic_decimator_4x_enhanced.v
    cdc_modules.v fir_lowpass.v ddc_input_interface.v
    chirp_memory_loader_param.v latency_buffer.v
    matched_filter_multi_segment.v matched_filter_processing_chain.v
    range_bin_decimator.v doppler_processor.v xfft_16.v fft_engine.v
    rx_gain_control.v mti_canceller.v
)

# Full system top (receiver chain + TX + USB + detection + self-test)
SYSTEM_RTL=(
    radar_system_top.v
    radar_transmitter.v dac_interface_single.v plfm_chirp_controller.v
    "${RECEIVER_RTL[@]}"
    usb_data_interface.v usb_data_interface_ft2232h.v edge_detector.v
    cfar_ca.v fpga_self_test.v
)

# ---- Layer A: iverilog -Wall compilation ----
run_lint_iverilog() {
    local label="$1"
    shift
    local files=("$@")
    local warn_file="/tmp/iverilog_lint_$$_${label}.log"

    printf "  %-45s " "iverilog -Wall ($label)"

    if ! iverilog -g2001 -DSIMULATION -Wall -o /dev/null "${files[@]}" 2>"$warn_file"; then
        # Hard compile error — always fatal
        echo -e "${RED}COMPILE ERROR${NC}"
        while IFS= read -r line; do
            echo "    $line"
        done < "$warn_file"
        LINT_ERR=$((LINT_ERR + 1))
        rm -f "$warn_file"
        return 1
    fi

    # Parse warnings — classify as error-level or info-level
    local err_count=0
    local info_count=0
    local err_lines=""

    while IFS= read -r line; do
        # Part-select out of range — Vivado Synth 8-524 (ERROR in Vivado)
        if echo "$line" | grep -q 'Part select.*is selecting after the vector\|out of bound bits'; then
            err_count=$((err_count + 1))
            err_lines="$err_lines\n    ${RED}[VIVADO-ERR]${NC} $line"
        # Port width mismatch / connection mismatch
        elif echo "$line" | grep -q 'port.*does not match\|Port.*mismatch'; then
            err_count=$((err_count + 1))
            err_lines="$err_lines\n    ${RED}[VIVADO-ERR]${NC} $line"
        # Informational warnings (timescale, dangling ports, array sensitivity)
        elif echo "$line" | grep -q 'timescale\|dangling\|sensitive to all'; then
            info_count=$((info_count + 1))
        # Unknown warning — report but don't fail
        elif [[ -n "$line" ]]; then
            info_count=$((info_count + 1))
        fi
    done < "$warn_file"

    if [[ "$err_count" -gt 0 ]]; then
        echo -e "${RED}FAIL${NC} ($err_count Vivado-class errors, $info_count info)"
        echo -e "$err_lines"
        LINT_ERR=$((LINT_ERR + err_count))
    else
        echo -e "${GREEN}PASS${NC} ($info_count info warnings)"
    fi

    rm -f "$warn_file"
}

# ---- Layer B: Custom regex static checks ----
# Catches patterns that Vivado treats as errors/warnings but iverilog ignores
run_lint_static() {
    printf "  %-45s " "Static RTL checks"

    local err_count=0
    local warn_count=0
    local err_lines=""
    local warn_lines=""

    for f in "$@"; do
        [[ -f "$f" ]] || continue
        # Skip testbench files (tb/ directory) — only lint production RTL
        case "$f" in tb/*) continue ;; esac

        local linenum=0
        while IFS= read -r line; do
            linenum=$((linenum + 1))

            # --- CHECK 1: Part-select with literal range on reg ---
            # Pattern: identifier[N:M] where N exceeds declared width
            # (iverilog catches this, but belt-and-suspenders)

            # --- CHECK 2: case/casex/casez without default (non-full case) ---
            # Vivado SYNTH-6 / inferred latch warning
            # Heuristic: look for case/casex/casez, then check if 'default' appears
            # before the matching 'endcase'. This is approximate — full parsing
            # would need a real parser. We flag 'case' lines so the developer
            # can manually verify.
            # (Handled below as a multi-line check)

            # --- CHECK 3: Blocking assignment (=) inside always @(posedge ...) ---
            # Vivado SYNTH-5 warning for inferred latches / race conditions
            # Only flag if the always block is clocked (posedge/negedge)
            # This is a heuristic — we check for '= ' that isn't '<=', '==', '!='
            # inside an always block header containing 'posedge' or 'negedge'.
            # (Too complex for line-by-line — skip for now, handled by testbenches)

            # --- CHECK 4: Multi-driven register (assign + always on same signal) ---
            # (Would need cross-file analysis — skip for v1)

        done < "$f"
    done

    # --- Multi-line check: case without default ---
    for f in "$@"; do
        [[ -f "$f" ]] || continue
        case "$f" in tb/*) continue ;; esac

        # Find case blocks and check for default
        # Use awk to find case..endcase blocks missing 'default'
        local missing_defaults
        missing_defaults=$(awk '
            /^[[:space:]]*(case|casex|casez)[[:space:]]*\(/ {
                case_line = NR
                case_file = FILENAME
                has_default = 0
                in_case = 1
                next
            }
            in_case && /default[[:space:]]*:/ {
                has_default = 1
            }
            in_case && /endcase/ {
                if (!has_default) {
                    printf "%s:%d: case statement without default\n", FILENAME, case_line
                }
                in_case = 0
            }
        ' "$f" 2>/dev/null)

        if [[ -n "$missing_defaults" ]]; then
            while IFS= read -r hit; do
                warn_count=$((warn_count + 1))
                warn_lines="$warn_lines\n    ${YELLOW}[SYNTH-6]${NC} $hit"
            done <<< "$missing_defaults"
        fi
    done

    # CHECK 5 ($readmemh in synth code) and CHECK 6 (unused includes)
    # require multi-line ifdef tracking / cross-file analysis. Not feasible
    # with line-by-line regex. Omitted — use Vivado lint instead.

    if [[ "$err_count" -gt 0 ]]; then
        echo -e "${RED}FAIL${NC} ($err_count errors, $warn_count warnings)"
        echo -e "$err_lines"
        LINT_ERR=$((LINT_ERR + err_count))
    elif [[ "$warn_count" -gt 0 ]]; then
        echo -e "${YELLOW}WARN${NC} ($warn_count warnings)"
        echo -e "$warn_lines"
        LINT_WARN=$((LINT_WARN + warn_count))
    else
        echo -e "${GREEN}PASS${NC}"
    fi
}

# ---------------------------------------------------------------------------
# Helper: compile and run a single testbench
#   run_test <name> <vvp_path> <iverilog_args...>
# ---------------------------------------------------------------------------
run_test() {
    local name="$1"
    local vvp="$2"
    shift 2
    local args=("$@")

    printf "  %-45s " "$name"

    # Compile
    if ! iverilog -g2001 -DSIMULATION -o "$vvp" "${args[@]}" 2>/tmp/iverilog_err_$$; then
        echo -e "${RED}COMPILE FAIL${NC}"
        ERRORS="$ERRORS\n  $name: compile error ($(head -1 /tmp/iverilog_err_$$))"
        FAIL=$((FAIL + 1))
        return
    fi

    # Run
    local output
    output=$("$TIMEOUT_CMD" 120 vvp "$vvp" 2>&1) || true

    # Count PASS/FAIL in output (testbenches use explicit [PASS]/[FAIL] markers)
    local test_pass test_fail
    test_pass=$(echo "$output" | grep -Ec '^\[PASS([^]]*)\]' || true)
    test_fail=$(echo "$output" | grep -Ec '^\[FAIL([^]]*)\]' || true)

    if [[ "$test_fail" -gt 0 ]]; then
        echo -e "${RED}FAIL${NC} (pass=$test_pass, fail=$test_fail)"
        ERRORS="$ERRORS\n  $name: $test_fail failure(s)"
        FAIL=$((FAIL + 1))
    elif [[ "$test_pass" -gt 0 ]]; then
        echo -e "${GREEN}PASS${NC} ($test_pass checks)"
        PASS=$((PASS + 1))
    else
        # No PASS/FAIL markers — check for clean completion
        if echo "$output" | grep -qi 'finish\|complete\|done'; then
            echo -e "${GREEN}PASS${NC} (completed)"
            PASS=$((PASS + 1))
        else
            echo -e "${YELLOW}UNKNOWN${NC} (no PASS/FAIL markers)"
            ERRORS="$ERRORS\n  $name: no pass/fail markers in output"
            FAIL=$((FAIL + 1))
        fi
    fi

    rm -f "$vvp"
}

# ===========================================================================
echo "============================================"
echo "  AERIS-10 FPGA Regression Test Suite"
echo "============================================"
echo ""
echo "Date: $(date)"
echo "iverilog: $(iverilog -V 2>&1 | head -1)"
echo ""

# ===========================================================================
# PHASE 0: LINT (Vivado-class error detection)
# ===========================================================================
if [[ "$SKIP_LINT" -eq 0 ]]; then
    echo "--- PHASE 0: LINT (Vivado-class checks) ---"

    # Layer A: iverilog -Wall on full production design
    run_lint_iverilog "production" "${PROD_RTL[@]}"

    # Layer A: standalone modules not in top-level hierarchy
    for extra in "${EXTRA_RTL[@]}"; do
        if [[ -f "$extra" ]]; then
            run_lint_iverilog "$(basename "$extra" .v)" "$extra"
        fi
    done

    # Layer B: custom static regex checks
    ALL_RTL=("${PROD_RTL[@]}" "${EXTRA_RTL[@]}")
    run_lint_static "${ALL_RTL[@]}"

    echo ""
    if [[ "$LINT_ERR" -gt 0 ]]; then
        echo -e "${RED}  LINT FAILED: $LINT_ERR Vivado-class error(s) detected.${NC}"
        echo "  Fix lint errors before pushing to Vivado. Aborting regression."
        echo ""
        exit 1
    elif [[ "$LINT_WARN" -gt 0 ]]; then
        echo -e "${YELLOW}  LINT: $LINT_WARN advisory warning(s) (non-blocking)${NC}"
    else
        echo -e "${GREEN}  LINT: All checks passed${NC}"
    fi
    echo ""
else
    echo "--- PHASE 0: LINT (skipped via --skip-lint) ---"
    echo ""
fi

# ===========================================================================
# PHASE 1: UNIT TESTS — Changed Modules (HIGH PRIORITY)
# ===========================================================================
echo "--- PHASE 1: Changed Modules ---"

run_test "CIC Decimator" \
    tb/tb_cic_reg.vvp \
    tb/tb_cic_decimator.v cic_decimator_4x_enhanced.v

run_test "Chirp Controller (BRAM)" \
    tb/tb_chirp_reg.vvp \
    tb/tb_chirp_controller.v plfm_chirp_controller.v

run_test "Chirp Contract" \
    tb/tb_chirp_ctr_reg.vvp \
    tb/tb_chirp_contract.v plfm_chirp_controller.v

run_test "Doppler Processor (DSP48)" \
    tb/tb_doppler_reg.vvp \
    tb/tb_doppler_cosim.v doppler_processor.v xfft_16.v fft_engine.v

run_test "Threshold Detector (detection bugs)" \
    tb/tb_threshold_detector.vvp \
    tb/tb_threshold_detector.v

run_test "RX Gain Control (digital gain)" \
    tb/tb_rx_gain_control.vvp \
    tb/tb_rx_gain_control.v rx_gain_control.v

run_test "MTI Canceller (ground clutter)" \
    tb/tb_mti_canceller.vvp \
    tb/tb_mti_canceller.v mti_canceller.v

run_test "CFAR CA Detector" \
    tb/tb_cfar_ca.vvp \
    tb/tb_cfar_ca.v cfar_ca.v

run_test "FPGA Self-Test" \
    tb/tb_fpga_self_test.vvp \
    tb/tb_fpga_self_test.v fpga_self_test.v

echo ""

# ===========================================================================
# PHASE 2: INTEGRATION TESTS
# ===========================================================================
echo "--- PHASE 2: Integration Tests ---"

run_test "DDC Chain (NCO→CIC→FIR)" \
    tb/tb_ddc_reg.vvp \
    tb/tb_ddc_cosim.v ddc_400m.v nco_400m_enhanced.v \
    cic_decimator_4x_enhanced.v fir_lowpass.v cdc_modules.v

# Real-data co-simulation: committed golden hex vs RTL (exact match required).
# These catch architecture mismatches (e.g. 32-pt → dual 16-pt Doppler FFT)
# that self-blessing golden-generate/compare tests cannot detect.
run_test "Doppler Real-Data (ADI CN0566, exact match)" \
    tb/tb_doppler_realdata.vvp \
    tb/tb_doppler_realdata.v doppler_processor.v xfft_16.v fft_engine.v

run_test "Full-Chain Real-Data (decim→Doppler, exact match)" \
    tb/tb_fullchain_realdata.vvp \
    tb/tb_fullchain_realdata.v range_bin_decimator.v \
    doppler_processor.v xfft_16.v fft_engine.v

if [[ "$QUICK" -eq 0 ]]; then
    # Golden generate
    run_test "Receiver (golden generate)" \
        tb/tb_rx_golden_reg.vvp \
        -DGOLDEN_GENERATE \
        tb/tb_radar_receiver_final.v "${RECEIVER_RTL[@]}"

    # Golden compare
    run_test "Receiver (golden compare)" \
        tb/tb_rx_compare_reg.vvp \
        tb/tb_radar_receiver_final.v "${RECEIVER_RTL[@]}"

    # Full system top (monitoring-only, legacy)
    run_test "System Top (radar_system_tb)" \
        tb/tb_system_reg.vvp \
        tb/radar_system_tb.v "${SYSTEM_RTL[@]}"

    # E2E integration (46 strict checks: TX, RX, USB R/W, CDC, safety, reset)
    run_test "System E2E (tb_system_e2e)" \
        tb/tb_system_e2e_reg.vvp \
        tb/tb_system_e2e.v "${SYSTEM_RTL[@]}"

    # USB_MODE=1 (FT2232H production) variants of system tests
    run_test "System Top USB_MODE=1 (FT2232H)" \
        tb/tb_system_ft2232h_reg.vvp \
        -DUSB_MODE_1 \
        tb/radar_system_tb.v "${SYSTEM_RTL[@]}"

    run_test "System E2E USB_MODE=1 (FT2232H)" \
        tb/tb_system_e2e_ft2232h_reg.vvp \
        -DUSB_MODE_1 \
        tb/tb_system_e2e.v "${SYSTEM_RTL[@]}"
else
    echo "  (skipped receiver golden + system top + E2E — use without --quick)"
    SKIP=$((SKIP + 6))
fi

echo ""

# ===========================================================================
# PHASE 3: UNIT TESTS — Signal Processing
# ===========================================================================
echo "--- PHASE 3: Signal Processing ---"

run_test "FFT Engine" \
    tb/tb_fft_reg.vvp \
    tb/tb_fft_engine.v fft_engine.v

run_test "NCO 400MHz" \
    tb/tb_nco_reg.vvp \
    tb/tb_nco_400m.v nco_400m_enhanced.v

run_test "FIR Lowpass" \
    tb/tb_fir_reg.vvp \
    tb/tb_fir_lowpass.v fir_lowpass.v

run_test "Matched Filter Chain" \
    tb/tb_mf_reg.vvp \
    tb/tb_matched_filter_processing_chain.v matched_filter_processing_chain.v \
    fft_engine.v chirp_memory_loader_param.v

echo ""

# ===========================================================================
# PHASE 4: UNIT TESTS — Infrastructure
# ===========================================================================
echo "--- PHASE 4: Infrastructure ---"

run_test "CDC Modules (3 variants)" \
    tb/tb_cdc_reg.vvp \
    tb/tb_cdc_modules.v cdc_modules.v

run_test "Edge Detector" \
    tb/tb_edge_reg.vvp \
    tb/tb_edge_detector.v edge_detector.v

run_test "USB Data Interface" \
    tb/tb_usb_reg.vvp \
    tb/tb_usb_data_interface.v usb_data_interface.v

run_test "Range Bin Decimator" \
    tb/tb_rbd_reg.vvp \
    tb/tb_range_bin_decimator.v range_bin_decimator.v

run_test "Radar Mode Controller" \
    tb/tb_rmc_reg.vvp \
    tb/tb_radar_mode_controller.v radar_mode_controller.v

echo ""

# ===========================================================================
# SUMMARY
# ===========================================================================
TOTAL=$((PASS + FAIL + SKIP))
echo "============================================"
echo "  RESULTS"
echo "============================================"
if [[ "$SKIP_LINT" -eq 0 ]]; then
    if [[ "$LINT_ERR" -gt 0 ]]; then
        echo -e "  Lint:  ${RED}$LINT_ERR error(s)${NC}, $LINT_WARN warning(s)"
    elif [[ "$LINT_WARN" -gt 0 ]]; then
        echo -e "  Lint:  ${GREEN}0 errors${NC}, ${YELLOW}$LINT_WARN warning(s)${NC}"
    else
        echo -e "  Lint:  ${GREEN}clean${NC}"
    fi
fi
echo "  Tests: $PASS passed, $FAIL failed, $SKIP skipped / $TOTAL total"
echo "============================================"

if [[ -n "$ERRORS" ]]; then
    echo ""
    echo "Failures:"
    echo -e "$ERRORS"
fi

echo ""

# Exit with error if any failures
if [[ "$FAIL" -gt 0 ]]; then
    exit 1
fi

exit 0
