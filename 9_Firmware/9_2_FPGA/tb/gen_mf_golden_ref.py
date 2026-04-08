#!/usr/bin/env python3
"""
Generate golden reference hex files for testing the matched_filter_processing_chain
Verilog module.

Matched filter operation: output = IFFT( FFT(signal) * conj(FFT(reference)) )

Test cases:
  Case 1: DC autocorrelation
  Case 2: Tone autocorrelation (bin 5)
  Case 3: Shifted tone cross-correlation (bin 5, 3-sample delay)
  Case 4: Impulse autocorrelation

Each case produces 6 hex files (sig_i, sig_q, ref_i, ref_q, out_i, out_q)
plus a human-readable summary file.

Usage:
  cd /Users/ganeshpanth/PLFM_RADAR/9_Firmware/9_2_FPGA/tb
  python3 gen_mf_golden_ref.py
"""

import os

import numpy as np

N = 1024  # FFT length


def to_q15(value):
    """Clamp a floating-point value to 16-bit signed range [-32768, 32767]."""
    v = int(np.round(value))
    v = max(-32768, min(32767, v))
    return v


def to_hex16(value):
    """Convert a 16-bit signed integer to 4-char hex string (two's complement)."""
    v = to_q15(value)
    if v < 0:
        v += 65536  # two's complement for negative
    return f"{v:04X}"


def write_hex_file(filepath, data):
    """Write an array of 16-bit signed values as 4-digit hex, one per line."""
    with open(filepath, "w") as f:
        for val in data:
            f.write(to_hex16(val) + "\n")


def matched_filter(sig_i, sig_q, ref_i, ref_q):
    """
    Compute matched filter output:
      output = IFFT( FFT(signal) * conj(FFT(reference)) )
    Returns (out_i, out_q) as float arrays.
    """
    signal_complex = sig_i.astype(np.float64) + 1j * sig_q.astype(np.float64)
    ref_complex = ref_i.astype(np.float64) + 1j * ref_q.astype(np.float64)

    S = np.fft.fft(signal_complex)
    R = np.fft.fft(ref_complex)

    product = S * np.conj(R)
    result = np.fft.ifft(product)

    out_i = np.real(result)
    out_q = np.imag(result)
    return out_i, out_q


def quantize_16bit(arr):
    """Quantize float array to 16-bit signed, clamped to [-32768, 32767]."""
    return np.array([to_q15(v) for v in arr], dtype=np.int32)


def generate_case(case_num, sig_i, sig_q, ref_i, ref_q, description, outdir):
    """Generate all hex files for one test case. Returns summary info."""
    # Compute matched filter
    out_i_f, out_q_f = matched_filter(sig_i, sig_q, ref_i, ref_q)

    # Quantize output
    out_i_q = quantize_16bit(out_i_f)
    out_q_q = quantize_16bit(out_q_f)

    # Find peak bin
    magnitude = np.sqrt(out_i_f**2 + out_q_f**2)
    peak_bin = int(np.argmax(magnitude))
    peak_mag_float = magnitude[peak_bin]
    peak_i = out_i_f[peak_bin]
    peak_q = out_q_f[peak_bin]
    peak_i_q = out_i_q[peak_bin]
    peak_q_q = out_q_q[peak_bin]

    # Write hex files
    prefix = os.path.join(outdir, "mf_golden")
    write_hex_file(f"{prefix}_sig_i_case{case_num}.hex", sig_i)
    write_hex_file(f"{prefix}_sig_q_case{case_num}.hex", sig_q)
    write_hex_file(f"{prefix}_ref_i_case{case_num}.hex", ref_i)
    write_hex_file(f"{prefix}_ref_q_case{case_num}.hex", ref_q)
    write_hex_file(f"{prefix}_out_i_case{case_num}.hex", out_i_q)
    write_hex_file(f"{prefix}_out_q_case{case_num}.hex", out_q_q)

    files = [
        f"mf_golden_sig_i_case{case_num}.hex",
        f"mf_golden_sig_q_case{case_num}.hex",
        f"mf_golden_ref_i_case{case_num}.hex",
        f"mf_golden_ref_q_case{case_num}.hex",
        f"mf_golden_out_i_case{case_num}.hex",
        f"mf_golden_out_q_case{case_num}.hex",
    ]

    summary = {
        "case": case_num,
        "description": description,
        "peak_bin": peak_bin,
        "peak_mag_float": peak_mag_float,
        "peak_i_float": peak_i,
        "peak_q_float": peak_q,
        "peak_i_quant": peak_i_q,
        "peak_q_quant": peak_q_q,
        "files": files,
    }
    return summary


def main():
    outdir = os.path.dirname(os.path.abspath(__file__))
    summaries = []
    all_files = []

    # =========================================================================
    # Case 1: DC autocorrelation
    # Signal and reference: I=0x1000 (4096), Q=0x0000 for all 1024 samples
    # FFT of DC signal: bin 0 = N*4096, bins 1..N-1 = 0
    # Product = |FFT(sig)|^2 at bin 0, zero elsewhere
    # IFFT: DC energy at bin 0 = N * 4096^2 / N = 4096^2 = 16777216 (will clamp)
    # =========================================================================
    sig_i = np.full(N, 0x1000, dtype=np.float64)  # 4096
    sig_q = np.zeros(N, dtype=np.float64)
    ref_i = np.full(N, 0x1000, dtype=np.float64)
    ref_q = np.zeros(N, dtype=np.float64)
    s = generate_case(
        1,
        sig_i,
        sig_q,
        ref_i,
        ref_q,
        "DC autocorrelation: signal=ref=DC(I=0x1000,Q=0). "
        "Expected: large peak at bin 0, zero elsewhere. "
        "Peak will saturate to 32767 due to 16-bit clamp.",
        outdir,
    )
    summaries.append(s)
    all_files.extend(s["files"])

    # =========================================================================
    # Case 2: Tone autocorrelation at bin 5
    # Signal and reference: complex tone at bin 5, amplitude 8000 (Q15)
    # sig[n] = 8000 * exp(j * 2*pi*5*n/N)
    # Autocorrelation of a tone => peak at bin 0 (lag 0)
    # =========================================================================
    amp = 8000.0
    k = 5
    n = np.arange(N, dtype=np.float64)
    tone = amp * np.exp(1j * 2 * np.pi * k * n / N)
    sig_i = np.round(np.real(tone)).astype(np.float64)
    sig_q = np.round(np.imag(tone)).astype(np.float64)
    ref_i = sig_i.copy()
    ref_q = sig_q.copy()
    s = generate_case(
        2,
        sig_i,
        sig_q,
        ref_i,
        ref_q,
        "Tone autocorrelation: signal=ref=tone(bin 5, amp 8000). "
        "Expected: peak at bin 0 (autocorrelation peak at zero lag).",
        outdir,
    )
    summaries.append(s)
    all_files.extend(s["files"])

    # =========================================================================
    # Case 3: Shifted tone cross-correlation
    # Signal: tone at bin 5
    # Reference: same tone at bin 5 but delayed by 3 samples
    # Cross-correlation peak should appear shifted from bin 0
    # =========================================================================
    delay = 3
    tone_sig = amp * np.exp(1j * 2 * np.pi * k * n / N)
    tone_ref = amp * np.exp(1j * 2 * np.pi * k * (n - delay) / N)
    sig_i = np.round(np.real(tone_sig)).astype(np.float64)
    sig_q = np.round(np.imag(tone_sig)).astype(np.float64)
    ref_i = np.round(np.real(tone_ref)).astype(np.float64)
    ref_q = np.round(np.imag(tone_ref)).astype(np.float64)
    s = generate_case(
        3,
        sig_i,
        sig_q,
        ref_i,
        ref_q,
        f"Shifted tone: signal=tone(bin 5), ref=tone(bin 5) delayed "
        f"by {delay} samples. Cross-correlation peak should shift to "
        f"indicate the delay.",
        outdir,
    )
    summaries.append(s)
    all_files.extend(s["files"])

    # =========================================================================
    # Case 4: Impulse autocorrelation
    # Signal: delta at sample 0 (I=0x7FFF=32767, Q=0)
    # Reference: same delta
    # FFT(delta) = flat spectrum (all bins = 32767)
    # Product = |32767|^2 at every bin
    # IFFT => scaled delta at sample 0
    # IFFT result[0] = N * 32767^2 / N = 32767^2 = ~1.07e9 => clamps to 32767
    # All other bins: 0
    # =========================================================================
    sig_i = np.zeros(N, dtype=np.float64)
    sig_q = np.zeros(N, dtype=np.float64)
    ref_i = np.zeros(N, dtype=np.float64)
    ref_q = np.zeros(N, dtype=np.float64)
    sig_i[0] = 32767.0  # 0x7FFF
    ref_i[0] = 32767.0
    s = generate_case(
        4,
        sig_i,
        sig_q,
        ref_i,
        ref_q,
        "Impulse autocorrelation: signal=ref=delta(n=0, I=0x7FFF). "
        "Expected: scaled delta at bin 0 (will saturate to 32767). "
        "All other bins should be zero.",
        outdir,
    )
    summaries.append(s)
    all_files.extend(s["files"])

    # =========================================================================
    # Write summary file
    # =========================================================================
    summary_path = os.path.join(outdir, "mf_golden_summary.txt")
    with open(summary_path, "w") as f:
        f.write("=" * 72 + "\n")
        f.write("Matched Filter Golden Reference Summary\n")
        f.write("Operation: output = IFFT( FFT(signal) * conj(FFT(reference)) )\n")
        f.write(f"FFT length: {N}\n")
        f.write("=" * 72 + "\n\n")

        for s in summaries:
            f.write("-" * 72 + "\n")
            f.write(f"Case {s['case']}: {s['description']}\n")
            f.write("-" * 72 + "\n")
            f.write(f"  Peak bin:              {s['peak_bin']}\n")
            f.write(f"  Peak magnitude (float):{s['peak_mag_float']:.6f}\n")
            f.write(f"  Peak I (float):        {s['peak_i_float']:.6f}\n")
            f.write(f"  Peak Q (float):        {s['peak_q_float']:.6f}\n")
            f.write(f"  Peak I (quantized):    {s['peak_i_quant']}\n")
            f.write(f"  Peak Q (quantized):    {s['peak_q_quant']}\n")
            f.write("  Files:\n")
            for fname in s["files"]:
                f.write(f"    {fname}\n")
            f.write("\n")

    all_files.append("mf_golden_summary.txt")

    # =========================================================================
    # Print summary to stdout
    # =========================================================================
    print("=" * 72)
    print("Matched Filter Golden Reference Generator")
    print(f"Output directory: {outdir}")
    print(f"FFT length: {N}")
    print("=" * 72)

    for s in summaries:
        print()
        print(f"Case {s['case']}: {s['description']}")
        print(f"  Peak bin:              {s['peak_bin']}")
        print(f"  Peak magnitude (float):{s['peak_mag_float']:.6f}")
        print(f"  Peak I (float):        {s['peak_i_float']:.6f}")
        print(f"  Peak Q (float):        {s['peak_q_float']:.6f}")
        print(f"  Peak I (quantized):    {s['peak_i_quant']}")
        print(f"  Peak Q (quantized):    {s['peak_q_quant']}")

    print()
    print(f"Generated {len(all_files)} files:")
    for fname in all_files:
        print(f"  {fname}")
    print()
    print("Done.")


if __name__ == "__main__":
    main()
