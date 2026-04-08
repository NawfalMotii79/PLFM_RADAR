#!/usr/bin/env python3
"""
Generate matched-filter co-simulation golden reference data.

Uses the bit-accurate Python model (fpga_model.py) to compute the expected
matched filter output for the bb_mf_test + ref_chirp test vectors.

Also generates additional test cases (DC, impulse, tone) for completeness.

The RTL testbench (tb_mf_cosim.v) runs the same inputs through the
SIMULATION-branch behavioral FFT in matched_filter_processing_chain.v.
compare_mf.py then compares the two.

Usage:
    cd ~/PLFM_RADAR/9_Firmware/9_2_FPGA/tb/cosim
    python3 gen_mf_cosim_golden.py

Author: Phase 0.5 matched-filter co-simulation suite for PLFM_RADAR
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fpga_model import MatchedFilterChain, saturate, sign_extend

FFT_SIZE = 1024


def load_hex_16bit(filepath):
    """Load 16-bit hex file (one value per line, with optional // comments)."""
    values = []
    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("//"):
                continue
            val = int(line, 16)
            values.append(sign_extend(val, 16))
    return values


def write_hex_16bit(filepath, data):
    """Write list of signed 16-bit integers as 4-digit hex, one per line."""
    with open(filepath, "w") as f:
        for val in data:
            v = val & 0xFFFF
            f.write(f"{v:04X}\n")


def write_csv(filepath, col_names, *columns):
    """Write CSV with header and columns."""
    with open(filepath, "w") as f:
        f.write(",".join(col_names) + "\n")
        n = len(columns[0])
        for i in range(n):
            row = ",".join(str(col[i]) for col in columns)
            f.write(row + "\n")


def generate_case(case_name, sig_i, sig_q, ref_i, ref_q, description, outdir, write_inputs=False):
    """
    Run matched filter through Python model and save golden output.

    If write_inputs=True, also writes the input hex files that the RTL
    testbench expects (mf_sig_<case>_i.hex, mf_sig_<case>_q.hex,
    mf_ref_<case>_i.hex, mf_ref_<case>_q.hex).

    Returns dict with case info and results.
    """
    print(f"\n--- {case_name}: {description} ---")

    assert len(sig_i) == FFT_SIZE, f"sig_i length {len(sig_i)} != {FFT_SIZE}"
    assert len(sig_q) == FFT_SIZE
    assert len(ref_i) == FFT_SIZE
    assert len(ref_q) == FFT_SIZE

    # Write input hex files for RTL testbench if requested
    if write_inputs:
        write_hex_16bit(os.path.join(outdir, f"mf_sig_{case_name}_i.hex"), sig_i)
        write_hex_16bit(os.path.join(outdir, f"mf_sig_{case_name}_q.hex"), sig_q)
        write_hex_16bit(os.path.join(outdir, f"mf_ref_{case_name}_i.hex"), ref_i)
        write_hex_16bit(os.path.join(outdir, f"mf_ref_{case_name}_q.hex"), ref_q)
        print(f"  Wrote input hex: mf_sig_{case_name}_{{i,q}}.hex, mf_ref_{case_name}_{{i,q}}.hex")

    # Run through bit-accurate Python model
    mf = MatchedFilterChain(fft_size=FFT_SIZE)
    out_i, out_q = mf.process(sig_i, sig_q, ref_i, ref_q)

    # Find peak
    peak_mag = -1
    peak_bin = 0
    for k in range(FFT_SIZE):
        mag = abs(out_i[k]) + abs(out_q[k])
        if mag > peak_mag:
            peak_mag = mag
            peak_bin = k

    print(f"  Output: {len(out_i)} samples")
    print(f"  Peak bin: {peak_bin}, magnitude: {peak_mag}")
    print(f"  Peak I={out_i[peak_bin]}, Q={out_q[peak_bin]}")

    # Save golden output hex
    write_hex_16bit(os.path.join(outdir, f"mf_golden_py_i_{case_name}.hex"), out_i)
    write_hex_16bit(os.path.join(outdir, f"mf_golden_py_q_{case_name}.hex"), out_q)

    # Save golden output CSV for comparison
    indices = list(range(FFT_SIZE))
    write_csv(
        os.path.join(outdir, f"mf_golden_py_{case_name}.csv"),
        ["bin", "out_i", "out_q"],
        indices,
        out_i,
        out_q,
    )

    return {
        "case_name": case_name,
        "description": description,
        "peak_bin": peak_bin,
        "peak_mag": peak_mag,
        "peak_i": out_i[peak_bin],
        "peak_q": out_q[peak_bin],
        "out_i": out_i,
        "out_q": out_q,
    }


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))

    print("=" * 60)
    print("Matched Filter Co-Sim Golden Reference Generator")
    print("Using bit-accurate Python model (fpga_model.py)")
    print("=" * 60)

    results = []

    # ---- Case 1: bb_mf_test + ref_chirp (realistic radar scenario) ----
    bb_i_path = os.path.join(base_dir, "bb_mf_test_i.hex")
    bb_q_path = os.path.join(base_dir, "bb_mf_test_q.hex")
    ref_i_path = os.path.join(base_dir, "ref_chirp_i.hex")
    ref_q_path = os.path.join(base_dir, "ref_chirp_q.hex")

    if all(os.path.exists(p) for p in [bb_i_path, bb_q_path, ref_i_path, ref_q_path]):
        bb_i = load_hex_16bit(bb_i_path)
        bb_q = load_hex_16bit(bb_q_path)
        ref_i = load_hex_16bit(ref_i_path)
        ref_q = load_hex_16bit(ref_q_path)
        r = generate_case(
            "chirp",
            bb_i,
            bb_q,
            ref_i,
            ref_q,
            "Radar chirp: 2 targets (500m, 1500m) vs ref chirp",
            base_dir,
        )
        results.append(r)
    else:
        print("\nWARNING: bb_mf_test / ref_chirp hex files not found.")
        print("Run radar_scene.py first.")

    # ---- Case 2: DC autocorrelation ----
    dc_val = 0x1000  # 4096
    sig_i = [dc_val] * FFT_SIZE
    sig_q = [0] * FFT_SIZE
    ref_i = [dc_val] * FFT_SIZE
    ref_q = [0] * FFT_SIZE
    r = generate_case(
        "dc",
        sig_i,
        sig_q,
        ref_i,
        ref_q,
        "DC autocorrelation: I=0x1000, Q=0",
        base_dir,
        write_inputs=True,
    )
    results.append(r)

    # ---- Case 3: Impulse autocorrelation ----
    sig_i = [0] * FFT_SIZE
    sig_q = [0] * FFT_SIZE
    ref_i = [0] * FFT_SIZE
    ref_q = [0] * FFT_SIZE
    sig_i[0] = 0x7FFF  # 32767
    ref_i[0] = 0x7FFF
    r = generate_case(
        "impulse",
        sig_i,
        sig_q,
        ref_i,
        ref_q,
        "Impulse autocorrelation: delta at n=0, I=0x7FFF",
        base_dir,
        write_inputs=True,
    )
    results.append(r)

    # ---- Case 4: Tone autocorrelation at bin 5 ----
    amp = 8000
    k = 5
    sig_i = []
    sig_q = []
    for n in range(FFT_SIZE):
        angle = 2.0 * math.pi * k * n / FFT_SIZE
        sig_i.append(saturate(int(round(amp * math.cos(angle))), 16))
        sig_q.append(saturate(int(round(amp * math.sin(angle))), 16))
    ref_i = list(sig_i)
    ref_q = list(sig_q)
    r = generate_case(
        "tone5",
        sig_i,
        sig_q,
        ref_i,
        ref_q,
        "Tone autocorrelation: bin 5, amplitude 8000",
        base_dir,
        write_inputs=True,
    )
    results.append(r)

    # ---- Summary ----
    print("\n" + "=" * 60)
    print("Summary:")
    print("=" * 60)
    for r in results:
        print(
            f"  {r['case_name']:10s}: peak at bin {r['peak_bin']}, "
            f"mag={r['peak_mag']}, I={r['peak_i']}, Q={r['peak_q']}"
        )

    print(f"\nGenerated {len(results)} golden reference cases.")
    print("Files written to:", base_dir)
    print("=" * 60)


if __name__ == "__main__":
    main()
