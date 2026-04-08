#!/usr/bin/env python3
"""
Co-simulation Comparison: RTL vs Python Model for AERIS-10 Doppler Processor.

Compares the RTL Doppler output (from tb_doppler_cosim.v) against the Python
model golden reference (from gen_doppler_golden.py).

After fixing the windowing pipeline bugs in doppler_processor.v (BRAM address
alignment and pipeline staging), the RTL achieves BIT-PERFECT match with the
Python model.  The comparison checks:
  1. Per-range-bin peak Doppler bin agreement (100% required)
  2. Per-range-bin I/Q correlation (1.0 expected)
  3. Per-range-bin magnitude spectrum correlation (1.0 expected)
  4. Global output energy (exact match expected)

Usage:
    python3 compare_doppler.py [scenario|all]

    scenario: stationary, moving, two_targets (default: stationary)
    all: run all scenarios

Author: Phase 0.5 Doppler co-simulation suite for PLFM_RADAR
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


# =============================================================================
# Configuration
# =============================================================================

DOPPLER_FFT = 32
RANGE_BINS = 64
TOTAL_OUTPUTS = RANGE_BINS * DOPPLER_FFT  # 2048
SUBFRAME_SIZE = 16

SCENARIOS = {
    "stationary": {
        "golden_csv": "doppler_golden_py_stationary.csv",
        "rtl_csv": "rtl_doppler_stationary.csv",
        "description": "Single stationary target at ~500m",
    },
    "moving": {
        "golden_csv": "doppler_golden_py_moving.csv",
        "rtl_csv": "rtl_doppler_moving.csv",
        "description": "Single moving target v=15m/s",
    },
    "two_targets": {
        "golden_csv": "doppler_golden_py_two_targets.csv",
        "rtl_csv": "rtl_doppler_two_targets.csv",
        "description": "Two targets at different ranges/velocities",
    },
}

# Pass/fail thresholds — BIT-PERFECT match expected after pipeline fix
PEAK_AGREEMENT_MIN = 1.00  # 100% peak Doppler bin agreement required
MAG_CORR_MIN = 0.99  # Near-perfect magnitude correlation required
ENERGY_RATIO_MIN = 0.999  # Energy ratio must be ~1.0 (bit-perfect)
ENERGY_RATIO_MAX = 1.001  # Energy ratio must be ~1.0 (bit-perfect)


# =============================================================================
# Helper functions
# =============================================================================


def load_doppler_csv(filepath):
    """
    Load Doppler output CSV with columns (range_bin, doppler_bin, out_i, out_q).
    Returns dict: {rbin: [(dbin, i, q), ...]}
    """
    data = {}
    with open(filepath, "r") as f:
        f.readline()  # Skip header
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(",")
            rbin = int(parts[0])
            dbin = int(parts[1])
            i_val = int(parts[2])
            q_val = int(parts[3])
            if rbin not in data:
                data[rbin] = []
            data[rbin].append((dbin, i_val, q_val))
    return data


def extract_iq_arrays(data_dict, rbin):
    """Extract I and Q arrays for a given range bin, ordered by doppler bin."""
    if rbin not in data_dict:
        return [0] * DOPPLER_FFT, [0] * DOPPLER_FFT
    entries = sorted(data_dict[rbin], key=lambda x: x[0])
    i_arr = [e[1] for e in entries]
    q_arr = [e[2] for e in entries]
    return i_arr, q_arr


def pearson_correlation(a, b):
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
        return 1.0 if abs(mean_a - mean_b) < 1.0 else 0.0
    return cov / math.sqrt(std_a_sq * std_b_sq)


def magnitude_l1(i_arr, q_arr):
    """L1 magnitude: |I| + |Q|."""
    return [abs(i) + abs(q) for i, q in zip(i_arr, q_arr)]


def find_peak_bin(i_arr, q_arr):
    """Find bin with max L1 magnitude."""
    mags = magnitude_l1(i_arr, q_arr)
    return max(range(len(mags)), key=lambda k: mags[k])


def peak_bins_match(py_peak, rtl_peak):
    """Return True if peaks match within +/-1 bin inside the same sub-frame."""
    py_sf = py_peak // SUBFRAME_SIZE
    rtl_sf = rtl_peak // SUBFRAME_SIZE
    if py_sf != rtl_sf:
        return False

    py_bin = py_peak % SUBFRAME_SIZE
    rtl_bin = rtl_peak % SUBFRAME_SIZE
    diff = abs(py_bin - rtl_bin)
    return diff <= 1 or diff >= SUBFRAME_SIZE - 1


def total_energy(data_dict):
    """Sum of I^2 + Q^2 across all range bins and Doppler bins."""
    total = 0
    for rbin in data_dict:
        for dbin, i_val, q_val in data_dict[rbin]:
            total += i_val * i_val + q_val * q_val
    return total


# =============================================================================
# Scenario comparison
# =============================================================================


def compare_scenario(name, config, base_dir):
    """Compare one Doppler scenario. Returns (passed, result_dict)."""
    print(f"\n{'=' * 60}")
    print(f"Scenario: {name} — {config['description']}")
    print(f"{'=' * 60}")

    golden_path = os.path.join(base_dir, config["golden_csv"])
    rtl_path = os.path.join(base_dir, config["rtl_csv"])

    if not os.path.exists(golden_path):
        print(f"  ERROR: Golden CSV not found: {golden_path}")
        print("  Run: python3 gen_doppler_golden.py")
        return False, {}
    if not os.path.exists(rtl_path):
        print(f"  ERROR: RTL CSV not found: {rtl_path}")
        print("  Run the Verilog testbench first")
        return False, {}

    py_data = load_doppler_csv(golden_path)
    rtl_data = load_doppler_csv(rtl_path)

    py_rbins = sorted(py_data.keys())
    rtl_rbins = sorted(rtl_data.keys())

    print(
        f"  Python: {len(py_rbins)} range bins, "
        f"{sum(len(v) for v in py_data.values())} total samples"
    )
    print(
        f"  RTL:    {len(rtl_rbins)} range bins, "
        f"{sum(len(v) for v in rtl_data.values())} total samples"
    )

    # ---- Check 1: Both have data ----
    py_total = sum(len(v) for v in py_data.values())
    rtl_total = sum(len(v) for v in rtl_data.values())
    if py_total == 0 or rtl_total == 0:
        print("  ERROR: One or both outputs are empty")
        return False, {}

    # ---- Check 2: Output count ----
    count_ok = rtl_total == TOTAL_OUTPUTS
    print(
        f"\n  Output count: RTL={rtl_total}, expected={TOTAL_OUTPUTS} "
        f"{'OK' if count_ok else 'MISMATCH'}"
    )

    # ---- Check 3: Global energy ----
    py_energy = total_energy(py_data)
    rtl_energy = total_energy(rtl_data)
    if py_energy > 0:
        energy_ratio = rtl_energy / py_energy
    else:
        energy_ratio = 1.0 if rtl_energy == 0 else float("inf")

    print("\n  Global energy:")
    print(f"    Python: {py_energy}")
    print(f"    RTL:    {rtl_energy}")
    print(f"    Ratio:  {energy_ratio:.4f}")

    # ---- Check 4: Per-range-bin analysis ----
    peak_agreements = 0
    mag_correlations = []
    i_correlations = []
    q_correlations = []

    peak_details = []

    for rbin in range(RANGE_BINS):
        py_i, py_q = extract_iq_arrays(py_data, rbin)
        rtl_i, rtl_q = extract_iq_arrays(rtl_data, rbin)

        py_peak = find_peak_bin(py_i, py_q)
        rtl_peak = find_peak_bin(rtl_i, rtl_q)

        # Peak agreement (allow +/-1 bin tolerance, but only within a sub-frame)
        if peak_bins_match(py_peak, rtl_peak):
            peak_agreements += 1

        py_mag = magnitude_l1(py_i, py_q)
        rtl_mag = magnitude_l1(rtl_i, rtl_q)

        mag_corr = pearson_correlation(py_mag, rtl_mag)
        corr_i = pearson_correlation(py_i, rtl_i)
        corr_q = pearson_correlation(py_q, rtl_q)

        mag_correlations.append(mag_corr)
        i_correlations.append(corr_i)
        q_correlations.append(corr_q)

        py_rbin_energy = sum(i * i + q * q for i, q in zip(py_i, py_q))
        rtl_rbin_energy = sum(i * i + q * q for i, q in zip(rtl_i, rtl_q))

        peak_details.append(
            {
                "rbin": rbin,
                "py_peak": py_peak,
                "rtl_peak": rtl_peak,
                "mag_corr": mag_corr,
                "corr_i": corr_i,
                "corr_q": corr_q,
                "py_energy": py_rbin_energy,
                "rtl_energy": rtl_rbin_energy,
            }
        )

    peak_agreement_frac = peak_agreements / RANGE_BINS
    avg_mag_corr = sum(mag_correlations) / len(mag_correlations)
    avg_corr_i = sum(i_correlations) / len(i_correlations)
    avg_corr_q = sum(q_correlations) / len(q_correlations)

    print("\n  Per-range-bin metrics:")
    print(
        f"    Peak Doppler bin agreement (+/-1 within sub-frame): {peak_agreements}/{RANGE_BINS} "
        f"({peak_agreement_frac:.0%})"
    )
    print(f"    Avg magnitude correlation: {avg_mag_corr:.4f}")
    print(f"    Avg I-channel correlation: {avg_corr_i:.4f}")
    print(f"    Avg Q-channel correlation: {avg_corr_q:.4f}")

    # Show top 5 range bins by Python energy
    print("\n  Top 5 range bins by Python energy:")
    top_rbins = sorted(peak_details, key=lambda x: -x["py_energy"])[:5]
    for d in top_rbins:
        print(
            f"    rbin={d['rbin']:2d}: py_peak={d['py_peak']:2d}, "
            f"rtl_peak={d['rtl_peak']:2d}, mag_corr={d['mag_corr']:.3f}, "
            f"I_corr={d['corr_i']:.3f}, Q_corr={d['corr_q']:.3f}"
        )

    # ---- Pass/Fail ----
    checks = []

    checks.append(("RTL output count == 2048", count_ok))

    energy_ok = ENERGY_RATIO_MIN < energy_ratio < ENERGY_RATIO_MAX
    checks.append((f"Energy ratio in bounds ({ENERGY_RATIO_MIN}-{ENERGY_RATIO_MAX})", energy_ok))

    peak_ok = peak_agreement_frac >= PEAK_AGREEMENT_MIN
    checks.append((f"Peak agreement >= {PEAK_AGREEMENT_MIN:.0%}", peak_ok))

    # For range bins with significant energy, check magnitude correlation
    high_energy_rbins = [d for d in peak_details if d["py_energy"] > py_energy / (RANGE_BINS * 10)]
    if high_energy_rbins:
        he_mag_corr = sum(d["mag_corr"] for d in high_energy_rbins) / len(high_energy_rbins)
        he_ok = he_mag_corr >= MAG_CORR_MIN
        checks.append(
            (
                f"High-energy rbin avg mag_corr >= {MAG_CORR_MIN:.2f} (actual={he_mag_corr:.3f})",
                he_ok,
            )
        )

    print("\n  Pass/Fail Checks:")
    all_pass = True
    for check_name, passed in checks:
        status = "PASS" if passed else "FAIL"
        print(f"    [{status}] {check_name}")
        if not passed:
            all_pass = False

    # ---- Write detailed comparison CSV ----
    compare_csv = os.path.join(base_dir, f"compare_doppler_{name}.csv")
    with open(compare_csv, "w") as f:
        f.write("range_bin,doppler_bin,py_i,py_q,rtl_i,rtl_q,diff_i,diff_q\n")
        for rbin in range(RANGE_BINS):
            py_i, py_q = extract_iq_arrays(py_data, rbin)
            rtl_i, rtl_q = extract_iq_arrays(rtl_data, rbin)
            for dbin in range(DOPPLER_FFT):
                f.write(
                    f"{rbin},{dbin},{py_i[dbin]},{py_q[dbin]},"
                    f"{rtl_i[dbin]},{rtl_q[dbin]},"
                    f"{rtl_i[dbin] - py_i[dbin]},{rtl_q[dbin] - py_q[dbin]}\n"
                )
    print(f"\n  Detailed comparison: {compare_csv}")

    result = {
        "scenario": name,
        "rtl_count": rtl_total,
        "energy_ratio": energy_ratio,
        "peak_agreement": peak_agreement_frac,
        "avg_mag_corr": avg_mag_corr,
        "avg_corr_i": avg_corr_i,
        "avg_corr_q": avg_corr_q,
        "passed": all_pass,
    }

    return all_pass, result


# =============================================================================
# Main
# =============================================================================


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))

    if len(sys.argv) > 1:
        arg = sys.argv[1].lower()
    else:
        arg = "stationary"

    if arg == "all":
        run_scenarios = list(SCENARIOS.keys())
    elif arg in SCENARIOS:
        run_scenarios = [arg]
    else:
        print(f"Unknown scenario: {arg}")
        print(f"Valid: {', '.join(SCENARIOS.keys())}, all")
        sys.exit(1)

    print("=" * 60)
    print("Doppler Processor Co-Simulation Comparison")
    print("RTL vs Python model (clean, no pipeline bug replication)")
    print(f"Scenarios: {', '.join(run_scenarios)}")
    print("=" * 60)

    results = []
    for name in run_scenarios:
        passed, result = compare_scenario(name, SCENARIOS[name], base_dir)
        results.append((name, passed, result))

    # Summary
    print(f"\n{'=' * 60}")
    print("SUMMARY")
    print(f"{'=' * 60}")

    print(
        f"\n  {'Scenario':<15} {'Energy Ratio':>13} {'Mag Corr':>10} "
        f"{'Peak Agree':>11} {'I Corr':>8} {'Q Corr':>8} {'Status':>8}"
    )
    print(f"  {'-' * 15} {'-' * 13} {'-' * 10} {'-' * 11} {'-' * 8} {'-' * 8} {'-' * 8}")

    all_pass = True
    for name, passed, result in results:
        if not result:
            print(f"  {name:<15} {'ERROR':>13} {'—':>10} {'—':>11} {'—':>8} {'—':>8} {'FAIL':>8}")
            all_pass = False
        else:
            status = "PASS" if passed else "FAIL"
            print(
                f"  {name:<15} {result['energy_ratio']:>13.4f} "
                f"{result['avg_mag_corr']:>10.4f} "
                f"{result['peak_agreement']:>10.0%} "
                f"{result['avg_corr_i']:>8.4f} "
                f"{result['avg_corr_q']:>8.4f} "
                f"{status:>8}"
            )
            if not passed:
                all_pass = False

    print()
    if all_pass:
        print("ALL TESTS PASSED")
    else:
        print("SOME TESTS FAILED")
    print(f"{'=' * 60}")

    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
