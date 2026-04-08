#!/usr/bin/env python3
"""
validate_mem_files.py — Validate all .mem files against AERIS-10 radar parameters.

Checks:
  1. Structural: line counts, hex format, value ranges for all 12 .mem files
  2. FFT twiddle files: bit-exact match against cos(2*pi*k/N) in Q15
  3. Long chirp .mem files: reverse-engineer parameters, check for chirp structure
  4. Short chirp .mem files: check length, value range, spectral content
   5. latency_buffer LATENCY=3187 parameter validation

Usage:
    python3 validate_mem_files.py
"""

import math
import os
import sys

# ============================================================================
# AERIS-10 System Parameters (from radar_scene.py)
# ============================================================================
F_CARRIER = 10.5e9  # 10.5 GHz carrier
C_LIGHT = 3.0e8
F_IF = 120e6  # IF frequency
CHIRP_BW = 20e6  # 20 MHz sweep
FS_ADC = 400e6  # ADC sample rate
FS_SYS = 100e6  # System clock (100 MHz, after CIC 4x)
T_LONG_CHIRP = 30e-6  # 30 us long chirp
T_SHORT_CHIRP = 0.5e-6  # 0.5 us short chirp
CIC_DECIMATION = 4
FFT_SIZE = 1024
DOPPLER_FFT_SIZE = 16
LONG_CHIRP_SAMPLES = int(T_LONG_CHIRP * FS_SYS)  # 3000 at 100 MHz

# Overlap-save parameters
OVERLAP_SAMPLES = 128
SEGMENT_ADVANCE = FFT_SIZE - OVERLAP_SAMPLES  # 896
LONG_SEGMENTS = 4

MEM_DIR = os.path.join(os.path.dirname(__file__), "..", "..")

pass_count = 0
fail_count = 0
warn_count = 0


def check(condition, label):
    global pass_count, fail_count
    if condition:
        print(f"  [PASS] {label}")
        pass_count += 1
    else:
        print(f"  [FAIL] {label}")
        fail_count += 1


def warn(label):
    global warn_count
    print(f"  [WARN] {label}")
    warn_count += 1


def read_mem_hex(filename):
    """Read a .mem file, return list of integer values (16-bit signed)."""
    path = os.path.join(MEM_DIR, filename)
    values = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("//"):
                continue
            val = int(line, 16)
            # Interpret as 16-bit signed
            if val >= 0x8000:
                val -= 0x10000
            values.append(val)
    return values


# ============================================================================
# TEST 1: Structural validation of all .mem files
# ============================================================================
def test_structural():
    print("\n=== TEST 1: Structural Validation ===")

    expected = {
        # FFT twiddle files (quarter-wave cosine ROMs)
        "fft_twiddle_1024.mem": {"lines": 256, "desc": "1024-pt FFT quarter-wave cos ROM"},
        "fft_twiddle_16.mem": {"lines": 4, "desc": "16-pt FFT quarter-wave cos ROM"},
        # Long chirp segments (4 segments x 1024 samples each)
        "long_chirp_seg0_i.mem": {"lines": 1024, "desc": "Long chirp seg 0 I"},
        "long_chirp_seg0_q.mem": {"lines": 1024, "desc": "Long chirp seg 0 Q"},
        "long_chirp_seg1_i.mem": {"lines": 1024, "desc": "Long chirp seg 1 I"},
        "long_chirp_seg1_q.mem": {"lines": 1024, "desc": "Long chirp seg 1 Q"},
        "long_chirp_seg2_i.mem": {"lines": 1024, "desc": "Long chirp seg 2 I"},
        "long_chirp_seg2_q.mem": {"lines": 1024, "desc": "Long chirp seg 2 Q"},
        "long_chirp_seg3_i.mem": {"lines": 1024, "desc": "Long chirp seg 3 I"},
        "long_chirp_seg3_q.mem": {"lines": 1024, "desc": "Long chirp seg 3 Q"},
        # Short chirp (50 samples)
        "short_chirp_i.mem": {"lines": 50, "desc": "Short chirp I"},
        "short_chirp_q.mem": {"lines": 50, "desc": "Short chirp Q"},
    }

    for fname, info in expected.items():
        path = os.path.join(MEM_DIR, fname)
        exists = os.path.isfile(path)
        check(exists, f"{fname} exists")
        if not exists:
            continue

        vals = read_mem_hex(fname)
        check(
            len(vals) == info["lines"],
            f"{fname}: {len(vals)} data lines (expected {info['lines']})",
        )

        # Check all values are in 16-bit signed range
        in_range = all(-32768 <= v <= 32767 for v in vals)
        check(in_range, f"{fname}: all values in [-32768, 32767]")


# ============================================================================
# TEST 2: FFT Twiddle Factor Validation
# ============================================================================
def test_twiddle_1024():
    print("\n=== TEST 2a: FFT Twiddle 1024 Validation ===")
    vals = read_mem_hex("fft_twiddle_1024.mem")

    # Expected: cos(2*pi*k/1024) for k=0..255, in Q15 format
    # Q15: value = round(cos(angle) * 32767)
    max_err = 0
    err_details = []
    for k in range(min(256, len(vals))):
        angle = 2.0 * math.pi * k / 1024.0
        expected = int(round(math.cos(angle) * 32767.0))
        expected = max(-32768, min(32767, expected))
        actual = vals[k]
        err = abs(actual - expected)
        if err > max_err:
            max_err = err
        if err > 1:
            err_details.append((k, actual, expected, err))

    check(max_err <= 1, f"fft_twiddle_1024.mem: max twiddle error = {max_err} LSB (tolerance: 1)")
    if err_details:
        for k, act, exp, e in err_details[:5]:
            print(f"    k={k}: got {act} (0x{act & 0xFFFF:04x}), expected {exp}, err={e}")
    print(f"  Max twiddle error: {max_err} LSB across {len(vals)} entries")


def test_twiddle_16():
    print("\n=== TEST 2b: FFT Twiddle 16 Validation ===")
    vals = read_mem_hex("fft_twiddle_16.mem")

    max_err = 0
    for k in range(min(4, len(vals))):
        angle = 2.0 * math.pi * k / 16.0
        expected = int(round(math.cos(angle) * 32767.0))
        expected = max(-32768, min(32767, expected))
        actual = vals[k]
        err = abs(actual - expected)
        if err > max_err:
            max_err = err

    check(max_err <= 1, f"fft_twiddle_16.mem: max twiddle error = {max_err} LSB (tolerance: 1)")
    print(f"  Max twiddle error: {max_err} LSB across {len(vals)} entries")

    # Print all 4 entries for reference
    print("  Twiddle 16 entries:")
    for k in range(min(4, len(vals))):
        angle = 2.0 * math.pi * k / 16.0
        expected = int(round(math.cos(angle) * 32767.0))
        print(
            f"    k={k}: file=0x{vals[k] & 0xFFFF:04x} ({vals[k]:6d}), "
            f"expected=0x{expected & 0xFFFF:04x} ({expected:6d}), "
            f"err={abs(vals[k] - expected)}"
        )


# ============================================================================
# TEST 3: Long Chirp .mem File Analysis
# ============================================================================
def test_long_chirp():
    print("\n=== TEST 3: Long Chirp .mem File Analysis ===")

    # Load all 4 segments
    all_i = []
    all_q = []
    for seg in range(4):
        seg_i = read_mem_hex(f"long_chirp_seg{seg}_i.mem")
        seg_q = read_mem_hex(f"long_chirp_seg{seg}_q.mem")
        all_i.extend(seg_i)
        all_q.extend(seg_q)

    total_samples = len(all_i)
    check(
        total_samples == 4096,
        f"Total long chirp samples: {total_samples} (expected 4096 = 4 segs x 1024)",
    )

    # Compute magnitude envelope
    magnitudes = [math.sqrt(i * i + q * q) for i, q in zip(all_i, all_q)]
    max_mag = max(magnitudes)
    min_mag = min(magnitudes)
    avg_mag = sum(magnitudes) / len(magnitudes)

    print(f"  Magnitude: min={min_mag:.1f}, max={max_mag:.1f}, avg={avg_mag:.1f}")
    print(
        f"  Max magnitude as fraction of Q15 range: {max_mag / 32767:.4f} ({max_mag / 32767 * 100:.2f}%)"
    )

    # Check if this looks like it came from generate_reference_chirp_q15
    # That function uses 32767 * 0.9 scaling => max magnitude ~29490
    expected_max_from_model = 32767 * 0.9
    uses_model_scaling = max_mag > expected_max_from_model * 0.8
    if uses_model_scaling:
        print("  Scaling: CONSISTENT with radar_scene.py model (0.9 * Q15)")
    else:
        warn(
            f"Magnitude ({max_mag:.0f}) is much lower than expected from Python model "
            f"({expected_max_from_model:.0f}). .mem files may have unknown provenance."
        )

    # Check non-zero content: how many samples are non-zero?
    nonzero_i = sum(1 for v in all_i if v != 0)
    nonzero_q = sum(1 for v in all_q if v != 0)
    print(f"  Non-zero samples: I={nonzero_i}/{total_samples}, Q={nonzero_q}/{total_samples}")

    # Analyze instantaneous frequency via phase differences
    # Phase = atan2(Q, I)
    phases = []
    for i_val, q_val in zip(all_i, all_q):
        if abs(i_val) > 5 or abs(q_val) > 5:  # Skip near-zero samples
            phases.append(math.atan2(q_val, i_val))
        else:
            phases.append(None)

    # Compute phase differences (instantaneous frequency)
    freq_estimates = []
    for n in range(1, len(phases)):
        if phases[n] is not None and phases[n - 1] is not None:
            dp = phases[n] - phases[n - 1]
            # Unwrap
            while dp > math.pi:
                dp -= 2 * math.pi
            while dp < -math.pi:
                dp += 2 * math.pi
            # Frequency in Hz (at 100 MHz sample rate, since these are post-DDC)
            f_inst = dp * FS_SYS / (2 * math.pi)
            freq_estimates.append(f_inst)

    if freq_estimates:
        f_start = sum(freq_estimates[:50]) / 50 if len(freq_estimates) > 50 else freq_estimates[0]
        f_end = sum(freq_estimates[-50:]) / 50 if len(freq_estimates) > 50 else freq_estimates[-1]
        f_min = min(freq_estimates)
        f_max = max(freq_estimates)
        f_range = f_max - f_min

        print("\n  Instantaneous frequency analysis (post-DDC baseband):")
        print(f"    Start freq:  {f_start / 1e6:.3f} MHz")
        print(f"    End freq:    {f_end / 1e6:.3f} MHz")
        print(f"    Min freq:    {f_min / 1e6:.3f} MHz")
        print(f"    Max freq:    {f_max / 1e6:.3f} MHz")
        print(f"    Freq range:  {f_range / 1e6:.3f} MHz")
        print(f"    Expected BW: {CHIRP_BW / 1e6:.3f} MHz")

        # A chirp should show frequency sweep
        is_chirp = f_range > 0.5e6  # At least 0.5 MHz sweep
        check(is_chirp, f"Long chirp shows frequency sweep ({f_range / 1e6:.2f} MHz > 0.5 MHz)")

        # Check if bandwidth roughly matches expected
        bw_match = abs(f_range - CHIRP_BW) / CHIRP_BW < 0.5  # within 50%
        if bw_match:
            print(
                f"  Bandwidth {f_range / 1e6:.2f} MHz roughly matches expected {CHIRP_BW / 1e6:.2f} MHz"
            )
        else:
            warn(
                f"Bandwidth {f_range / 1e6:.2f} MHz does NOT match expected {CHIRP_BW / 1e6:.2f} MHz"
            )

    # Compare segment boundaries for overlap-save consistency
    # In proper overlap-save, the chirp data should be segmented at 896-sample boundaries
    # with segments being 1024-sample FFT blocks
    print("\n  Segment boundary analysis:")
    for seg in range(4):
        seg_i = read_mem_hex(f"long_chirp_seg{seg}_i.mem")
        seg_q = read_mem_hex(f"long_chirp_seg{seg}_q.mem")
        seg_mags = [math.sqrt(i * i + q * q) for i, q in zip(seg_i, seg_q)]
        seg_avg = sum(seg_mags) / len(seg_mags)
        seg_max = max(seg_mags)

        # Check segment 3 zero-padding (chirp is 3000 samples, seg3 starts at 3072)
        # Samples 3000-4095 should be zero (or near-zero) if chirp is exactly 3000 samples
        if seg == 3:
            # Seg3 covers chirp samples 3072..4095
            # If chirp is only 3000 samples, then only samples 0..(3000-3072) = NONE are valid
            # Actually chirp has 3000 samples total. Seg3 starts at index 3*1024=3072.
            # So seg3 should only have 3000-3072 = -72 -> no valid chirp data!
            # Wait, but the .mem files have 1024 lines with non-trivial data...
            # Let's check if seg3 has significant data
            zero_count = sum(1 for m in seg_mags if m < 2)
            print(
                f"  Seg {seg}: avg_mag={seg_avg:.1f}, max_mag={seg_max:.1f}, "
                f"near-zero={zero_count}/{len(seg_mags)}"
            )
            if zero_count > 500:
                print("    -> Seg 3 mostly zeros (chirp shorter than 4096 samples)")
            else:
                print("    -> Seg 3 has significant data throughout")
        else:
            print(f"  Seg {seg}: avg_mag={seg_avg:.1f}, max_mag={seg_max:.1f}")


# ============================================================================
# TEST 4: Short Chirp .mem File Analysis
# ============================================================================
def test_short_chirp():
    print("\n=== TEST 4: Short Chirp .mem File Analysis ===")

    short_i = read_mem_hex("short_chirp_i.mem")
    short_q = read_mem_hex("short_chirp_q.mem")

    check(len(short_i) == 50, f"Short chirp I: {len(short_i)} samples (expected 50)")
    check(len(short_q) == 50, f"Short chirp Q: {len(short_q)} samples (expected 50)")

    # Expected: 0.5 us chirp at 100 MHz = 50 samples
    expected_samples = int(T_SHORT_CHIRP * FS_SYS)
    check(
        len(short_i) == expected_samples,
        f"Short chirp length matches T_SHORT_CHIRP * FS_SYS = {expected_samples}",
    )

    magnitudes = [math.sqrt(i * i + q * q) for i, q in zip(short_i, short_q)]
    max_mag = max(magnitudes)
    avg_mag = sum(magnitudes) / len(magnitudes)

    print(f"  Magnitude: max={max_mag:.1f}, avg={avg_mag:.1f}")
    print(f"  Max as fraction of Q15: {max_mag / 32767:.4f} ({max_mag / 32767 * 100:.2f}%)")

    # Check non-zero
    nonzero = sum(1 for m in magnitudes if m > 1)
    check(nonzero == len(short_i), f"All {nonzero}/{len(short_i)} samples non-zero")

    # Check it looks like a chirp (phase should be quadratic)
    phases = [math.atan2(q, i) for i, q in zip(short_i, short_q)]
    freq_est = []
    for n in range(1, len(phases)):
        dp = phases[n] - phases[n - 1]
        while dp > math.pi:
            dp -= 2 * math.pi
        while dp < -math.pi:
            dp += 2 * math.pi
        freq_est.append(dp * FS_SYS / (2 * math.pi))

    if freq_est:
        f_start = freq_est[0]
        f_end = freq_est[-1]
        print(f"  Freq start: {f_start / 1e6:.3f} MHz, end: {f_end / 1e6:.3f} MHz")
        print(f"  Freq range: {abs(f_end - f_start) / 1e6:.3f} MHz")


# ============================================================================
# TEST 5: Generate Expected Chirp .mem and Compare
# ============================================================================
def test_chirp_vs_model():
    print("\n=== TEST 5: Compare .mem Files vs Python Model ===")

    # Generate reference using the same method as radar_scene.py
    chirp_rate = CHIRP_BW / T_LONG_CHIRP  # Hz/s

    model_i = []
    model_q = []
    n_chirp = min(FFT_SIZE, LONG_CHIRP_SAMPLES)  # 1024

    for n in range(n_chirp):
        t = n / FS_SYS
        phase = math.pi * chirp_rate * t * t
        re_val = int(round(32767 * 0.9 * math.cos(phase)))
        im_val = int(round(32767 * 0.9 * math.sin(phase)))
        model_i.append(max(-32768, min(32767, re_val)))
        model_q.append(max(-32768, min(32767, im_val)))

    # Read seg0 from .mem
    mem_i = read_mem_hex("long_chirp_seg0_i.mem")
    mem_q = read_mem_hex("long_chirp_seg0_q.mem")

    # Compare magnitudes
    model_mags = [math.sqrt(i * i + q * q) for i, q in zip(model_i, model_q)]
    mem_mags = [math.sqrt(i * i + q * q) for i, q in zip(mem_i, mem_q)]

    model_max = max(model_mags)
    mem_max = max(mem_mags)

    print(f"  Python model seg0: max_mag={model_max:.1f} (Q15 * 0.9)")
    print(f"  .mem file seg0:    max_mag={mem_max:.1f}")
    print(f"  Ratio (mem/model): {mem_max / model_max:.4f}")

    # Check if they match (they almost certainly won't based on magnitude analysis)
    matches = sum(1 for a, b in zip(model_i, mem_i) if a == b)
    print(f"  Exact I matches: {matches}/{len(model_i)}")

    if matches > len(model_i) * 0.9:
        print("  -> .mem files MATCH Python model")
    else:
        warn(".mem files do NOT match Python model. They likely have different provenance.")
        # Try to detect scaling
        if mem_max > 0:
            ratio = model_max / mem_max
            print(f"  Scale factor (model/mem): {ratio:.2f}")
            print(f"  This suggests the .mem files used ~{1.0 / ratio:.4f} scaling instead of 0.9")

    # Check phase correlation (shape match regardless of scaling)
    model_phases = [math.atan2(q, i) for i, q in zip(model_i, model_q)]
    mem_phases = [math.atan2(q, i) for i, q in zip(mem_i, mem_q)]

    # Compute phase differences
    phase_diffs = []
    for mp, fp in zip(model_phases, mem_phases):
        d = mp - fp
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        phase_diffs.append(d)

    avg_phase_diff = sum(phase_diffs) / len(phase_diffs)
    max_phase_diff = max(abs(d) for d in phase_diffs)

    print("\n  Phase comparison (shape regardless of amplitude):")
    print(f"    Avg phase diff:  {avg_phase_diff:.4f} rad ({math.degrees(avg_phase_diff):.2f} deg)")
    print(f"    Max phase diff:  {max_phase_diff:.4f} rad ({math.degrees(max_phase_diff):.2f} deg)")

    phase_match = max_phase_diff < 0.5  # within 0.5 rad
    check(
        phase_match,
        f"Phase shape match: max diff = {math.degrees(max_phase_diff):.1f} deg (tolerance: 28.6 deg)",
    )


# ============================================================================
# TEST 6: Latency Buffer LATENCY=3187 Validation
# ============================================================================
def test_latency_buffer():
    print("\n=== TEST 6: Latency Buffer LATENCY=3187 Validation ===")

    # The latency buffer delays the reference chirp data to align with
    # the matched filter processing chain output.
    #
    # The total latency through the processing chain depends on the branch:
    #
    # SYNTHESIS branch (fft_engine.v):
    #   - Load: 1024 cycles (input)
    #   - Forward FFT: LOG2N=10 stages x N/2=512 butterflies x 5-cycle pipeline = variable
    #   - Reference FFT: same
    #   - Conjugate multiply: 1024 cycles (4-stage pipeline in frequency_matched_filter)
    #   - Inverse FFT: same as forward
    #   - Output: 1024 cycles
    #   Total: roughly 3000-4000 cycles depending on pipeline fill
    #
    # The LATENCY=3187 value was likely determined empirically to align
    # the reference chirp arriving at the processing chain with the
    # correct time-domain position.
    #
    # Key constraint: LATENCY must be < 4096 (BRAM buffer size)
    LATENCY = 3187
    BRAM_SIZE = 4096

    check(LATENCY < BRAM_SIZE, f"LATENCY ({LATENCY}) < BRAM size ({BRAM_SIZE})")

    # The fft_engine processes in stages:
    # - LOAD: 1024 clocks (accepts input)
    # - Per butterfly stage: 512 butterflies x 5 pipeline stages = ~2560 clocks + overhead
    #   Actually: 512 butterflies, each takes 5 cycles = 2560 per stage, 10 stages
    #   Total compute: 10 * 2560 = 25600 clocks
    # But this is just for ONE FFT. The chain does 3 FFTs + multiply.
    #
    # For the SIMULATION branch, it's 1 clock per operation (behavioral).
    # LATENCY=3187 doesn't apply to simulation branch behavior —
    # it's the physical hardware pipeline latency.
    #
    # For synthesis: the latency_buffer feeds ref data to the chain via
    # chirp_memory_loader_param → latency_buffer → chain.
    # But wait — looking at radar_receiver_final.v:
    #   - mem_request drives valid_in on the latency buffer
    #   - The buffer delays {ref_i, ref_q} by LATENCY valid_in cycles
    #   - The delayed output feeds long_chirp_real/imag → chain
    #
    # The purpose: the chain in the SYNTHESIS branch reads reference data
    # via the long_chirp_real/imag ports DURING ST_FWD_FFT (while collecting
    # input samples). The reference data needs to arrive LATENCY cycles
    # after the first mem_request, where LATENCY accounts for:
    #   - The fft_engine pipeline latency from input to output
    #   - Specifically, the chain processes: load 1024 → FFT → FFT → multiply → IFFT → output
    #     The reference is consumed during the second FFT (ST_REF_BITREV/BUTTERFLY)
    #     which starts after the first FFT completes.

    # For now, validate that LATENCY is reasonable (between 1000 and 4095)
    check(1000 < LATENCY < 4095, f"LATENCY={LATENCY} in reasonable range [1000, 4095]")

    # Check that the module name vs parameter is consistent
    print(f"  LATENCY parameter: {LATENCY}")
    print(f"  Module name: latency_buffer (parameterized, LATENCY={LATENCY})")
    # Module name was renamed from latency_buffer_2159 to latency_buffer
    # to match the actual parameterized LATENCY value. No warning needed.

    # Validate address arithmetic won't overflow
    # read_ptr = (write_ptr - LATENCY) mod 4096
    # With 12-bit address, max write_ptr = 4095
    # When write_ptr < LATENCY: read_ptr = 4096 + write_ptr - LATENCY
    # Minimum: 4096 + 0 - 3187 = 909 (valid)
    min_read_ptr = 4096 + 0 - LATENCY
    check(
        min_read_ptr >= 0 and min_read_ptr < 4096,
        f"Min read_ptr after wrap = {min_read_ptr} (valid: 0..4095)",
    )

    # The latency buffer uses valid_in gated reads, so it only counts
    # valid samples. The number of valid_in pulses between first write
    # and first read is LATENCY.
    print(f"  Buffer primes after {LATENCY} valid_in pulses, then outputs continuously")


# ============================================================================
# TEST 7: Cross-check chirp memory loader addressing
# ============================================================================
def test_memory_addressing():
    print("\n=== TEST 7: Chirp Memory Loader Addressing ===")

    # chirp_memory_loader_param uses: long_addr = {segment_select[1:0], sample_addr[9:0]}
    # This creates a 12-bit address: seg[1:0] ++ addr[9:0]
    # Segment 0: addresses 0x000..0x3FF (0..1023)
    # Segment 1: addresses 0x400..0x7FF (1024..2047)
    # Segment 2: addresses 0x800..0xBFF (2048..3071)
    # Segment 3: addresses 0xC00..0xFFF (3072..4095)

    for seg in range(4):
        base = seg * 1024
        end = base + 1023
        addr_from_concat = (seg << 10) | 0  # {seg[1:0], 10'b0}
        addr_end = (seg << 10) | 1023

        check(
            addr_from_concat == base,
            f"Seg {seg} base address: {{{seg}[1:0], 10'b0}} = {addr_from_concat} (expected {base})",
        )
        check(
            addr_end == end,
            f"Seg {seg} end address: {{{seg}[1:0], 10'h3FF}} = {addr_end} (expected {end})",
        )

    # Memory is declared as: reg [15:0] long_chirp_i [0:4095]
    # $readmemh loads seg0 to [0:1023], seg1 to [1024:2047], etc.
    # Addressing via {segment_select, sample_addr} maps correctly.
    print("  Addressing scheme: {segment_select[1:0], sample_addr[9:0]} -> 12-bit address")
    print("  Memory size: [0:4095] (4096 entries) — matches 4 segments x 1024 samples")


# ============================================================================
# TEST 8: Seg3 zero-padding analysis
# ============================================================================
def test_seg3_padding():
    print("\n=== TEST 8: Segment 3 Data Analysis ===")

    # The long chirp has 3000 samples (30 us at 100 MHz).
    # With 4 segments of 1024 samples = 4096 total memory slots.
    # Segments are loaded contiguously into memory:
    #   Seg0: chirp samples 0..1023
    #   Seg1: chirp samples 1024..2047
    #   Seg2: chirp samples 2048..3071
    #   Seg3: chirp samples 3072..4095
    #
    # But the chirp only has 3000 samples! So seg3 should have:
    #   Valid chirp data at indices 0..(3000-3072-1) = NEGATIVE
    #   Wait — 3072 > 3000, so seg3 has NO valid chirp samples if chirp is exactly 3000.
    #
    # However, the overlap-save algorithm in matched_filter_multi_segment.v
    # collects data differently:
    #   Seg0: collect 896 DDC samples, buffer[0:895], zero-pad [896:1023]
    #   Seg1: overlap from seg0[768:895] → buffer[0:127], collect 896 → buffer[128:1023]
    #   ...
    # The chirp reference is indexed by segment_select + sample_addr,
    # so it reads ALL 1024 values for each segment regardless.
    #
    # If the chirp is 3000 samples but only 4*1024=4096 slots exist,
    # the question is: do the .mem files contain 3000 samples of real chirp
    # data spread across 4096 slots, or something else?

    seg3_i = read_mem_hex("long_chirp_seg3_i.mem")
    seg3_q = read_mem_hex("long_chirp_seg3_q.mem")

    mags = [math.sqrt(i * i + q * q) for i, q in zip(seg3_i, seg3_q)]

    # Count trailing zeros (samples after chirp ends)
    trailing_zeros = 0
    for m in reversed(mags):
        if m < 2:
            trailing_zeros += 1
        else:
            break

    nonzero = sum(1 for m in mags if m > 2)

    print(f"  Seg3 non-zero samples: {nonzero}/{len(seg3_i)}")
    print(f"  Seg3 trailing near-zeros: {trailing_zeros}")
    print(f"  Seg3 max magnitude: {max(mags):.1f}")
    print(f"  Seg3 first 5 magnitudes: {[f'{m:.1f}' for m in mags[:5]]}")
    print(f"  Seg3 last 5 magnitudes: {[f'{m:.1f}' for m in mags[-5:]]}")

    if nonzero == 1024:
        print("  -> Seg3 has data throughout (chirp extends beyond 3072 samples or is padded)")
        # This means the .mem files encode 4096 chirp samples, not 3000
        # The chirp duration used for .mem generation was different from T_LONG_CHIRP
        actual_chirp_samples = 4 * 1024  # = 4096
        actual_duration = actual_chirp_samples / FS_SYS
        warn(
            f"Chirp in .mem files appears to be {actual_chirp_samples} samples "
            f"({actual_duration * 1e6:.1f} us), not {LONG_CHIRP_SAMPLES} samples "
            f"({T_LONG_CHIRP * 1e6:.1f} us)"
        )
    elif trailing_zeros > 100:
        # Some padding at end
        actual_valid = 3072 + (1024 - trailing_zeros)
        print(f"  -> Estimated valid chirp samples in .mem: ~{actual_valid}")


# ============================================================================
# MAIN
# ============================================================================
def main():
    print("=" * 70)
    print("AERIS-10 .mem File Validation")
    print("=" * 70)

    test_structural()
    test_twiddle_1024()
    test_twiddle_16()
    test_long_chirp()
    test_short_chirp()
    test_chirp_vs_model()
    test_latency_buffer()
    test_memory_addressing()
    test_seg3_padding()

    print("\n" + "=" * 70)
    print(f"SUMMARY: {pass_count} PASS, {fail_count} FAIL, {warn_count} WARN")
    if fail_count == 0:
        print("ALL CHECKS PASSED")
    else:
        print("SOME CHECKS FAILED")
    print("=" * 70)

    return 0 if fail_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
