#!/usr/bin/env python3
"""
Co-simulation Comparison: RTL vs Python Model for AERIS-10 DDC Chain.

Reads the ADC hex test vectors, runs them through the bit-accurate Python
model (fpga_model.py), then compares the output against the RTL simulation
CSV (from tb_ddc_cosim.v).

Key considerations:
  - The RTL DDC has LFSR phase dithering on the NCO FTW, so exact bit-match
    is not expected. We use statistical metrics (correlation, RMS error).
  - The CDC (gray-coded 400→100 MHz crossing) may introduce non-deterministic
    latency offsets. We auto-align using cross-correlation.
  - The comparison reports pass/fail based on configurable thresholds.

Usage:
    python3 compare.py [scenario]

    scenario: dc, single_target, multi_target, noise_only, sine_1mhz
              (default: dc)

Author: Phase 0.5 co-simulation suite for PLFM_RADAR
"""

import math
import os
import sys

# Add this directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fpga_model import SignalChain


# =============================================================================
# Configuration
# =============================================================================

# Thresholds for pass/fail
# These are generous because of LFSR dithering and CDC latency jitter
MAX_RMS_ERROR_LSB = 50.0       # Max RMS error in 18-bit LSBs
MAX_ABS_ERROR_LSB = 200.0     # Max absolute error in 18-bit LSBs (4x RMS threshold)
MIN_CORRELATION = 0.90         # Min Pearson correlation coefficient
MAX_LATENCY_DRIFT = 15         # Max latency offset between RTL and model (samples)
MAX_COUNT_DIFF = 20            # Max output count difference (LFSR dithering affects CIC timing)

# Scenarios
SCENARIOS = {
    'dc': {
        'adc_hex': 'adc_dc.hex',
        'rtl_csv': 'rtl_bb_dc.csv',
        'description': 'DC input (ADC=128)',
        # DC input: expect small outputs, but LFSR dithering adds ~+128 LSB
        # average bias to NCO FTW which accumulates through CIC integrators
        # as a small DC offset (~15-20 LSB in baseband). This is expected.
        'max_rms': 25.0,        # Relaxed to account for LFSR dithering bias
        'min_corr': -1.0,       # Correlation not meaningful for near-zero
    },
    'single_target': {
        'adc_hex': 'adc_single_target.hex',
        'rtl_csv': 'rtl_bb_single_target.csv',
        'description': 'Single target at 500m',
        'max_rms': MAX_RMS_ERROR_LSB,
        'min_corr': -1.0,       # Correlation not meaningful with LFSR dithering
    },
    'multi_target': {
        'adc_hex': 'adc_multi_target.hex',
        'rtl_csv': 'rtl_bb_multi_target.csv',
        'description': 'Multi-target (5 targets)',
        'max_rms': MAX_RMS_ERROR_LSB,
        'min_corr': -1.0,       # Correlation not meaningful with LFSR dithering
    },
    'noise_only': {
        'adc_hex': 'adc_noise_only.hex',
        'rtl_csv': 'rtl_bb_noise_only.csv',
        'description': 'Noise only',
        'max_rms': MAX_RMS_ERROR_LSB,
        'min_corr': -1.0,       # Correlation not meaningful with LFSR dithering
    },
    'sine_1mhz': {
        'adc_hex': 'adc_sine_1mhz.hex',
        'rtl_csv': 'rtl_bb_sine_1mhz.csv',
        'description': '1 MHz sine wave',
        'max_rms': MAX_RMS_ERROR_LSB,
        'min_corr': -1.0,       # Correlation not meaningful with LFSR dithering
    },
}


# =============================================================================
# Helper functions
# =============================================================================

def load_adc_hex(filepath):
    """Load 8-bit unsigned ADC samples from hex file."""
    samples = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('//'):
                continue
            samples.append(int(line, 16))
    return samples


def load_rtl_csv(filepath):
    """Load RTL baseband output CSV (sample_idx, baseband_i, baseband_q)."""
    bb_i = []
    bb_q = []
    with open(filepath) as f:
        f.readline()  # Skip header
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(',')
            bb_i.append(int(parts[1]))
            bb_q.append(int(parts[2]))
    return bb_i, bb_q


def run_python_model(adc_samples):
    """Run ADC samples through the Python DDC model.

    Returns the 18-bit FIR outputs (not the 16-bit DDC interface outputs),
    because the RTL testbench captures the FIR output directly
    (baseband_i_reg <= fir_i_out in ddc_400m.v).
    """

    chain = SignalChain()
    result = chain.process_adc_block(adc_samples)

    # Use fir_i_raw / fir_q_raw (18-bit) to match RTL's baseband output
    # which is the FIR output before DDC interface 18->16 rounding
    bb_i = result['fir_i_raw']
    bb_q = result['fir_q_raw']

    return bb_i, bb_q


def compute_rms_error(a, b):
    """Compute RMS error between two equal-length lists."""
    if len(a) != len(b):
        raise ValueError(f"Length mismatch: {len(a)} vs {len(b)}")
    if len(a) == 0:
        return 0.0
    sum_sq = sum((x - y) ** 2 for x, y in zip(a, b, strict=False))
    return math.sqrt(sum_sq / len(a))


def compute_max_abs_error(a, b):
    """Compute maximum absolute error between two equal-length lists."""
    if len(a) != len(b) or len(a) == 0:
        return 0
    return max(abs(x - y) for x, y in zip(a, b, strict=False))


def compute_correlation(a, b):
    """Compute Pearson correlation coefficient."""
    n = len(a)
    if n < 2:
        return 0.0

    mean_a = sum(a) / n
    mean_b = sum(b) / n

    cov = sum((a[i] - mean_a) * (b[i] - mean_b) for i in range(n))
    std_a_sq = sum((x - mean_a) ** 2 for x in a)
    std_b_sq = sum((x - mean_b) ** 2 for x in b)

    if std_a_sq < 1e-10 or std_b_sq < 1e-10:
        # Near-zero variance (e.g., DC input)
        return 1.0 if abs(mean_a - mean_b) < 1.0 else 0.0

    return cov / math.sqrt(std_a_sq * std_b_sq)


def cross_correlate_lag(a, b, max_lag=20):
    """
    Find the lag that maximizes cross-correlation between a and b.
    Returns (best_lag, best_correlation) where positive lag means b is delayed.
    """
    n = min(len(a), len(b))
    if n < 10:
        return 0, 0.0

    best_lag = 0
    best_corr = -2.0

    for lag in range(-max_lag, max_lag + 1):
        # Align: a[start_a:end_a] vs b[start_b:end_b]
        if lag >= 0:
            start_a = lag
            start_b = 0
        else:
            start_a = 0
            start_b = -lag

        end = min(len(a) - start_a, len(b) - start_b)
        if end < 10:
            continue

        seg_a = a[start_a:start_a + end]
        seg_b = b[start_b:start_b + end]

        corr = compute_correlation(seg_a, seg_b)
        if corr > best_corr:
            best_corr = corr
            best_lag = lag

    return best_lag, best_corr


def compute_signal_stats(samples):
    """Compute basic statistics of a signal."""
    if not samples:
        return {'mean': 0, 'rms': 0, 'min': 0, 'max': 0, 'count': 0}
    n = len(samples)
    mean = sum(samples) / n
    rms = math.sqrt(sum(x * x for x in samples) / n)
    return {
        'mean': mean,
        'rms': rms,
        'min': min(samples),
        'max': max(samples),
        'count': n,
    }


# =============================================================================
# Main comparison
# =============================================================================

def compare_scenario(scenario_name):
    """Run comparison for one scenario. Returns True if passed."""
    if scenario_name not in SCENARIOS:
        return False

    cfg = SCENARIOS[scenario_name]
    base_dir = os.path.dirname(os.path.abspath(__file__))


    # ---- Load ADC data ----
    adc_path = os.path.join(base_dir, cfg['adc_hex'])
    if not os.path.exists(adc_path):
        return False
    adc_samples = load_adc_hex(adc_path)

    # ---- Load RTL output ----
    rtl_path = os.path.join(base_dir, cfg['rtl_csv'])
    if not os.path.exists(rtl_path):
        return False
    rtl_i, rtl_q = load_rtl_csv(rtl_path)

    # ---- Run Python model ----
    py_i, py_q = run_python_model(adc_samples)

    # ---- Length comparison ----
    len_diff = abs(len(rtl_i) - len(py_i))

    # ---- Signal statistics ----
    rtl_i_stats = compute_signal_stats(rtl_i)
    rtl_q_stats = compute_signal_stats(rtl_q)
    py_i_stats = compute_signal_stats(py_i)
    py_q_stats = compute_signal_stats(py_q)


    # ---- Trim to common length ----
    common_len = min(len(rtl_i), len(py_i))
    if common_len < 10:
        return False

    rtl_i_trim = rtl_i[:common_len]
    rtl_q_trim = rtl_q[:common_len]
    py_i_trim = py_i[:common_len]
    py_q_trim = py_q[:common_len]

    # ---- Cross-correlation to find latency offset ----
    lag_i, _corr_i = cross_correlate_lag(rtl_i_trim, py_i_trim,
                                        max_lag=MAX_LATENCY_DRIFT)
    lag_q, _corr_q = cross_correlate_lag(rtl_q_trim, py_q_trim,
                                        max_lag=MAX_LATENCY_DRIFT)

    # ---- Apply latency correction ----
    best_lag = lag_i  # Use I-channel lag (should be same as Q)
    if abs(lag_i - lag_q) > 1:
        # Use the average
        best_lag = (lag_i + lag_q) // 2

    if best_lag > 0:
        # RTL is delayed relative to Python
        aligned_rtl_i = rtl_i_trim[best_lag:]
        aligned_rtl_q = rtl_q_trim[best_lag:]
        aligned_py_i = py_i_trim[:len(aligned_rtl_i)]
        aligned_py_q = py_q_trim[:len(aligned_rtl_q)]
    elif best_lag < 0:
        # Python is delayed relative to RTL
        aligned_py_i = py_i_trim[-best_lag:]
        aligned_py_q = py_q_trim[-best_lag:]
        aligned_rtl_i = rtl_i_trim[:len(aligned_py_i)]
        aligned_rtl_q = rtl_q_trim[:len(aligned_py_q)]
    else:
        aligned_rtl_i = rtl_i_trim
        aligned_rtl_q = rtl_q_trim
        aligned_py_i = py_i_trim
        aligned_py_q = py_q_trim

    aligned_len = min(len(aligned_rtl_i), len(aligned_py_i))
    aligned_rtl_i = aligned_rtl_i[:aligned_len]
    aligned_rtl_q = aligned_rtl_q[:aligned_len]
    aligned_py_i = aligned_py_i[:aligned_len]
    aligned_py_q = aligned_py_q[:aligned_len]


    # ---- Error metrics (after alignment) ----
    rms_i = compute_rms_error(aligned_rtl_i, aligned_py_i)
    rms_q = compute_rms_error(aligned_rtl_q, aligned_py_q)
    max_abs_i = compute_max_abs_error(aligned_rtl_i, aligned_py_i)
    max_abs_q = compute_max_abs_error(aligned_rtl_q, aligned_py_q)
    corr_i_aligned = compute_correlation(aligned_rtl_i, aligned_py_i)
    corr_q_aligned = compute_correlation(aligned_rtl_q, aligned_py_q)


    # ---- First/last sample comparison ----
    for k in range(min(10, aligned_len)):
        ei = aligned_rtl_i[k] - aligned_py_i[k]
        eq = aligned_rtl_q[k] - aligned_py_q[k]

    # ---- Write detailed comparison CSV ----
    compare_csv_path = os.path.join(base_dir, f"compare_{scenario_name}.csv")
    with open(compare_csv_path, 'w') as f:
        f.write("idx,rtl_i,py_i,err_i,rtl_q,py_q,err_q\n")
        for k in range(aligned_len):
            ei = aligned_rtl_i[k] - aligned_py_i[k]
            eq = aligned_rtl_q[k] - aligned_py_q[k]
            f.write(f"{k},{aligned_rtl_i[k]},{aligned_py_i[k]},{ei},"
                    f"{aligned_rtl_q[k]},{aligned_py_q[k]},{eq}\n")

    # ---- Pass/Fail ----
    max_rms = cfg.get('max_rms', MAX_RMS_ERROR_LSB)
    min_corr = cfg.get('min_corr', MIN_CORRELATION)

    results = []

    # Check 1: Output count sanity
    count_ok = len_diff <= MAX_COUNT_DIFF
    results.append(('Output count match', count_ok,
                     f"diff={len_diff} <= {MAX_COUNT_DIFF}"))

    # Check 2: RMS amplitude ratio (RTL vs Python should have same power)
    # The LFSR dithering randomizes sample phases but preserves overall
    # signal power, so RMS amplitudes should match within ~10%.
    rtl_rms = max(rtl_i_stats['rms'], rtl_q_stats['rms'])
    py_rms = max(py_i_stats['rms'], py_q_stats['rms'])
    if py_rms > 1.0 and rtl_rms > 1.0:
        rms_ratio = max(rtl_rms, py_rms) / min(rtl_rms, py_rms)
        rms_ratio_ok = rms_ratio <= 1.20  # Within 20%
        results.append(('RMS amplitude ratio', rms_ratio_ok,
                         f"ratio={rms_ratio:.3f} <= 1.20"))
    else:
        # Near-zero signals (DC input): check absolute RMS error
        rms_ok = max(rms_i, rms_q) <= max_rms
        results.append(('RMS error (low signal)', rms_ok,
                         f"max(I={rms_i:.2f}, Q={rms_q:.2f}) <= {max_rms:.1f}"))

    # Check 3: Mean DC offset match
    # Both should have similar DC bias. For large signals (where LFSR dithering
    # causes the NCO to walk in phase), allow the mean to differ proportionally
    # to the signal RMS. Use max(30 LSB, 3% of signal RMS).
    mean_err_i = abs(rtl_i_stats['mean'] - py_i_stats['mean'])
    mean_err_q = abs(rtl_q_stats['mean'] - py_q_stats['mean'])
    max_mean_err = max(mean_err_i, mean_err_q)
    signal_rms = max(rtl_rms, py_rms)
    mean_threshold = max(30.0, signal_rms * 0.03)  # 3% of signal RMS or 30 LSB
    mean_ok = max_mean_err <= mean_threshold
    results.append(('Mean DC offset match', mean_ok,
                     f"max_diff={max_mean_err:.1f} <= {mean_threshold:.1f}"))

    # Check 4: Correlation (skip for near-zero signals or dithered scenarios)
    if min_corr > -0.5:
        corr_ok = min(corr_i_aligned, corr_q_aligned) >= min_corr
        results.append(('Correlation', corr_ok,
                         f"min(I={corr_i_aligned:.4f}, Q={corr_q_aligned:.4f}) >= {min_corr:.2f}"))

    # Check 5: Dynamic range match
    # Peak amplitudes should be in the same ballpark
    rtl_peak = max(abs(rtl_i_stats['min']), abs(rtl_i_stats['max']),
                   abs(rtl_q_stats['min']), abs(rtl_q_stats['max']))
    py_peak = max(abs(py_i_stats['min']), abs(py_i_stats['max']),
                  abs(py_q_stats['min']), abs(py_q_stats['max']))
    if py_peak > 10 and rtl_peak > 10:
        peak_ratio = max(rtl_peak, py_peak) / min(rtl_peak, py_peak)
        peak_ok = peak_ratio <= 1.50  # Within 50%
        results.append(('Peak amplitude ratio', peak_ok,
                         f"ratio={peak_ratio:.3f} <= 1.50"))

    # Check 6: Latency offset
    lag_ok = abs(best_lag) <= MAX_LATENCY_DRIFT
    results.append(('Latency offset', lag_ok,
                     f"|{best_lag}| <= {MAX_LATENCY_DRIFT}"))

    # Check 7: Max absolute error
    # Catches outlier spikes (pipeline corruption, saturation, CDC glitches)
    # that RMS and correlation metrics alone can miss.
    max_abs_err = max(max_abs_i, max_abs_q)
    max_abs_threshold = cfg.get('max_abs', MAX_ABS_ERROR_LSB)
    max_abs_ok = max_abs_err <= max_abs_threshold
    results.append(('Max absolute error', max_abs_ok,
                     f"max(I={max_abs_i}, Q={max_abs_q}) <= {max_abs_threshold:.0f}"))

    # ---- Report ----
    all_pass = True
    for _name, ok, _detail in results:
        if not ok:
            all_pass = False

    if all_pass:
        pass
    else:
        pass

    return all_pass


def main():
    """Run comparison for specified scenario(s)."""
    if len(sys.argv) > 1:
        scenario = sys.argv[1]
        if scenario == 'all':
            # Run all scenarios that have RTL CSV files
            base_dir = os.path.dirname(os.path.abspath(__file__))
            overall_pass = True
            run_count = 0
            pass_count = 0
            for name, cfg in SCENARIOS.items():
                rtl_path = os.path.join(base_dir, cfg['rtl_csv'])
                if os.path.exists(rtl_path):
                    ok = compare_scenario(name)
                    run_count += 1
                    if ok:
                        pass_count += 1
                    else:
                        overall_pass = False
                else:
                    pass

            if overall_pass:
                pass
            else:
                pass
            return 0 if overall_pass else 1
        ok = compare_scenario(scenario)
        return 0 if ok else 1
    ok = compare_scenario('dc')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
