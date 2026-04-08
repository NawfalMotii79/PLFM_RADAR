#!/usr/bin/env python3
"""
Generate Doppler processor co-simulation golden reference data.

Uses the bit-accurate Python model (fpga_model.py) to compute the expected
Doppler FFT output for the dual 16-pt FFT architecture.  Also generates the
input hex files consumed by the Verilog testbench (tb_doppler_cosim.v).

Architecture:
  Sub-frame 0 (long PRI):  chirps 0-15  -> 16-pt Hamming -> 16-pt FFT -> bins 0-15
  Sub-frame 1 (short PRI): chirps 16-31 -> 16-pt Hamming -> 16-pt FFT -> bins 16-31

Usage:
    cd ~/PLFM_RADAR/9_Firmware/9_2_FPGA/tb/cosim
    python3 gen_doppler_golden.py
    python3 gen_doppler_golden.py stationary   # single scenario

Author: Phase 0.5 Doppler co-simulation suite for PLFM_RADAR
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fpga_model import DopplerProcessor
from radar_scene import Target, generate_doppler_frame

# =============================================================================
# Constants
# =============================================================================

DOPPLER_FFT_SIZE = 16  # Per sub-frame
DOPPLER_TOTAL_BINS = 32  # Total output (2 sub-frames x 16)
RANGE_BINS = 64
CHIRPS_PER_FRAME = 32
TOTAL_SAMPLES = CHIRPS_PER_FRAME * RANGE_BINS  # 2048


# =============================================================================
# I/O helpers
# =============================================================================


def write_hex_32bit(filepath, samples):
    """Write packed 32-bit hex file: {Q[31:16], I[15:0]} per line."""
    with open(filepath, "w") as f:
        f.write(f"// {len(samples)} packed 32-bit samples (Q:I) for $readmemh\n")
        for i_val, q_val in samples:
            packed = ((q_val & 0xFFFF) << 16) | (i_val & 0xFFFF)
            f.write(f"{packed:08X}\n")
    print(f"  Wrote {len(samples)} packed samples to {filepath}")


def write_csv(filepath, headers, *columns):
    """Write CSV with header row."""
    with open(filepath, "w") as f:
        f.write(",".join(headers) + "\n")
        for i in range(len(columns[0])):
            row = ",".join(str(col[i]) for col in columns)
            f.write(row + "\n")
    print(f"  Wrote {len(columns[0])} rows to {filepath}")


def write_hex_16bit(filepath, data):
    """Write list of signed 16-bit integers as 4-digit hex, one per line."""
    with open(filepath, "w") as f:
        for val in data:
            v = val & 0xFFFF
            f.write(f"{v:04X}\n")


# =============================================================================
# Buggy-model helpers  (match RTL pipeline misalignment)
# =============================================================================


# =============================================================================
# Test scenario definitions
# =============================================================================


def make_scenario_stationary():
    """Single stationary target at range bin ~10.  Doppler peak at bin 0."""
    targets = [Target(range_m=500, velocity_mps=0.0, rcs_dbsm=20.0)]
    return targets, "Single stationary target at ~500m (rbin~10), Doppler bin 0"


def make_scenario_moving():
    """Single target with moderate Doppler shift."""
    # v = 15 m/s → fd = 2*v*fc/c ≈ 1050 Hz
    # Long PRI = 167 us → sub-frame 0 bin = fd * 16 * 167e-6 ≈ 2.8 → bin ~3
    # Short PRI = 175 us → sub-frame 1 bin = fd * 16 * 175e-6 ≈ 2.9 → bin 16+3 = 19
    targets = [Target(range_m=500, velocity_mps=15.0, rcs_dbsm=20.0)]
    return targets, "Single moving target v=15m/s (~1050Hz Doppler, sf0 bin~3, sf1 bin~19)"


def make_scenario_two_targets():
    """Two targets at different ranges and velocities."""
    targets = [
        Target(range_m=300, velocity_mps=10.0, rcs_dbsm=20.0),
        Target(range_m=800, velocity_mps=-20.0, rcs_dbsm=15.0),
    ]
    return targets, "Two targets: 300m/+10m/s, 800m/-20m/s"


SCENARIOS = {
    "stationary": make_scenario_stationary,
    "moving": make_scenario_moving,
    "two_targets": make_scenario_two_targets,
}


# =============================================================================
# Main generator
# =============================================================================


def generate_scenario(name, targets, description, base_dir):
    """Generate input hex + golden output for one scenario."""
    print(f"\n{'=' * 60}")
    print(f"Scenario: {name} — {description}")
    print("Model: CLEAN (dual 16-pt FFT)")
    print(f"{'=' * 60}")

    # Generate Doppler frame (32 chirps x 64 range bins)
    frame_i, frame_q = generate_doppler_frame(targets, seed=42)

    print(f"  Generated frame: {len(frame_i)} chirps x {len(frame_i[0])} range bins")

    # ---- Write input hex file (packed 32-bit: {Q, I}) ----
    # RTL expects data streamed chirp-by-chirp: chirp0[rb0..rb63], chirp1[rb0..rb63], ...
    packed_samples = []
    for chirp in range(CHIRPS_PER_FRAME):
        for rb in range(RANGE_BINS):
            packed_samples.append((frame_i[chirp][rb], frame_q[chirp][rb]))

    input_hex = os.path.join(base_dir, f"doppler_input_{name}.hex")
    write_hex_32bit(input_hex, packed_samples)

    # ---- Run through Python model (dual 16-pt FFT) ----
    dp = DopplerProcessor()
    doppler_i, doppler_q = dp.process_frame(frame_i, frame_q)

    print(
        f"  Doppler output: {len(doppler_i)} range bins x "
        f"{len(doppler_i[0])} doppler bins (2 sub-frames x {DOPPLER_FFT_SIZE})"
    )

    # ---- Write golden output CSV ----
    # Format: range_bin, doppler_bin, out_i, out_q
    # Ordered same as RTL output: all doppler bins for rbin 0, then rbin 1, ...
    # Bins 0-15 = sub-frame 0 (long PRI), bins 16-31 = sub-frame 1 (short PRI)
    flat_rbin = []
    flat_dbin = []
    flat_i = []
    flat_q = []

    for rbin in range(RANGE_BINS):
        for dbin in range(DOPPLER_TOTAL_BINS):
            flat_rbin.append(rbin)
            flat_dbin.append(dbin)
            flat_i.append(doppler_i[rbin][dbin])
            flat_q.append(doppler_q[rbin][dbin])

    golden_csv = os.path.join(base_dir, f"doppler_golden_py_{name}.csv")
    write_csv(
        golden_csv,
        ["range_bin", "doppler_bin", "out_i", "out_q"],
        flat_rbin,
        flat_dbin,
        flat_i,
        flat_q,
    )

    # ---- Write golden hex (for optional RTL $readmemh comparison) ----
    golden_hex = os.path.join(base_dir, f"doppler_golden_py_{name}.hex")
    write_hex_32bit(golden_hex, list(zip(flat_i, flat_q)))

    # ---- Find peak per range bin ----
    print("\n  Peak Doppler bins per range bin (top 5 by magnitude):")
    peak_info = []
    for rbin in range(RANGE_BINS):
        mags = [
            abs(doppler_i[rbin][d]) + abs(doppler_q[rbin][d]) for d in range(DOPPLER_TOTAL_BINS)
        ]
        peak_dbin = max(range(DOPPLER_TOTAL_BINS), key=lambda d: mags[d])
        peak_mag = mags[peak_dbin]
        peak_info.append((rbin, peak_dbin, peak_mag))

    # Sort by magnitude descending, show top 5
    peak_info.sort(key=lambda x: -x[2])
    for rbin, dbin, mag in peak_info[:5]:
        i_val = doppler_i[rbin][dbin]
        q_val = doppler_q[rbin][dbin]
        sf = dbin // DOPPLER_FFT_SIZE
        bin_in_sf = dbin % DOPPLER_FFT_SIZE
        print(
            f"    rbin={rbin:2d}, dbin={dbin:2d} (sf{sf}:{bin_in_sf:2d}), mag={mag:6d}, "
            f"I={i_val:6d}, Q={q_val:6d}"
        )

    return {
        "name": name,
        "description": description,
        "peak_info": peak_info[:5],
    }


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))

    print("=" * 60)
    print("Doppler Processor Co-Sim Golden Reference Generator")
    print(f"Architecture: dual {DOPPLER_FFT_SIZE}-pt FFT ({DOPPLER_TOTAL_BINS} total bins)")
    print("=" * 60)

    scenarios_to_run = list(SCENARIOS.keys())

    # Check if a specific scenario was requested
    for arg in sys.argv[1:]:
        if arg.startswith("--"):
            continue
        if arg in SCENARIOS:
            scenarios_to_run = [arg]
            break

    results = []
    for name in scenarios_to_run:
        targets, description = SCENARIOS[name]()
        r = generate_scenario(name, targets, description, base_dir)
        results.append(r)

    print(f"\n{'=' * 60}")
    print("Summary:")
    print(f"{'=' * 60}")
    for r in results:
        print(
            f"  {r['name']:<15s} top peak: "
            f"rbin={r['peak_info'][0][0]}, dbin={r['peak_info'][0][1]}, "
            f"mag={r['peak_info'][0][2]}"
        )

    print(f"\nGenerated {len(results)} scenarios.")
    print(f"Files written to: {base_dir}")
    print("=" * 60)


if __name__ == "__main__":
    main()
