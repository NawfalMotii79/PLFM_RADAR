#!/usr/bin/env python3
"""
Synthetic Radar Scene Generator for AERIS-10 FPGA Co-simulation.

Generates test vectors (ADC samples + reference chirps) for multi-target
radar scenes with configurable:
  - Target range, velocity, RCS
  - Noise floor and clutter
  - ADC quantization (8-bit, 400 MSPS)

Output formats:
  - Hex files for Verilog $readmemh
  - CSV for analysis
  - Python arrays for direct use with fpga_model.py

The scene generator models the complete RF path:
  TX chirp -> propagation delay -> Doppler shift -> RX IF signal -> ADC

Author: Phase 0.5 co-simulation suite for PLFM_RADAR
"""

import math
import os

# =============================================================================
# AERIS-10 System Parameters
# =============================================================================

# RF parameters
F_CARRIER = 10.5e9  # 10.5 GHz carrier
C_LIGHT = 3.0e8  # Speed of light (m/s)
WAVELENGTH = C_LIGHT / F_CARRIER  # ~0.02857 m

# Chirp parameters
F_IF = 120e6  # IF frequency (120 MHz)
CHIRP_BW = 20e6  # Chirp bandwidth (30 MHz -> 10 MHz = 20 MHz sweep)
F_CHIRP_START = 30e6  # Chirp start frequency (relative to IF)
F_CHIRP_END = 10e6  # Chirp end frequency (relative to IF)

# Sampling
FS_ADC = 400e6  # ADC sample rate (400 MSPS)
FS_SYS = 100e6  # System clock (100 MHz)
ADC_BITS = 8  # ADC resolution

# Chirp timing
T_LONG_CHIRP = 30e-6  # 30 us long chirp duration
T_SHORT_CHIRP = 0.5e-6  # 0.5 us short chirp
T_LISTEN_LONG = 137e-6  # 137 us listening window
T_PRI_LONG = 167e-6  # 30 us chirp + 137 us listen
T_PRI_SHORT = 175e-6  # staggered short-PRI sub-frame
N_SAMPLES_LISTEN = int(T_LISTEN_LONG * FS_ADC)  # 54800 samples

# Processing chain
CIC_DECIMATION = 4
FFT_SIZE = 1024
RANGE_BINS = 64
DOPPLER_FFT_SIZE = 16  # Per sub-frame
DOPPLER_TOTAL_BINS = 32  # Total output bins (2 sub-frames x 16)
CHIRPS_PER_SUBFRAME = 16
CHIRPS_PER_FRAME = 32

# Derived
RANGE_RESOLUTION = C_LIGHT / (2 * CHIRP_BW)  # 7.5 m
MAX_UNAMBIGUOUS_RANGE = C_LIGHT * T_LISTEN_LONG / 2  # ~20.55 km
VELOCITY_RESOLUTION_LONG = WAVELENGTH / (2 * CHIRPS_PER_SUBFRAME * T_PRI_LONG)
VELOCITY_RESOLUTION_SHORT = WAVELENGTH / (2 * CHIRPS_PER_SUBFRAME * T_PRI_SHORT)

# Short chirp LUT (60 entries, 8-bit unsigned)
SHORT_CHIRP_LUT = [
    255,
    237,
    187,
    118,
    49,
    6,
    7,
    54,
    132,
    210,
    253,
    237,
    167,
    75,
    10,
    10,
    80,
    180,
    248,
    237,
    150,
    45,
    1,
    54,
    167,
    249,
    228,
    118,
    15,
    18,
    127,
    238,
    235,
    118,
    10,
    34,
    167,
    254,
    187,
    45,
    8,
    129,
    248,
    201,
    49,
    10,
    145,
    254,
    167,
    17,
    46,
    210,
    235,
    75,
    7,
    155,
    253,
    118,
    1,
    129,
]


# =============================================================================
# Target definition
# =============================================================================


class Target:
    """Represents a radar target."""

    def __init__(self, range_m, velocity_mps=0.0, rcs_dbsm=0.0, phase_deg=0.0):
        """
        Args:
            range_m: Target range in meters
            velocity_mps: Target radial velocity in m/s (positive = approaching)
            rcs_dbsm: Radar cross-section in dBsm
            phase_deg: Initial phase in degrees
        """
        self.range_m = range_m
        self.velocity_mps = velocity_mps
        self.rcs_dbsm = rcs_dbsm
        self.phase_deg = phase_deg

    @property
    def delay_s(self):
        """Round-trip delay in seconds."""
        return 2 * self.range_m / C_LIGHT

    @property
    def delay_samples(self):
        """Round-trip delay in ADC samples at 400 MSPS."""
        return self.delay_s * FS_ADC

    @property
    def doppler_hz(self):
        """Doppler frequency shift in Hz."""
        return 2 * self.velocity_mps * F_CARRIER / C_LIGHT

    @property
    def amplitude(self):
        """Linear amplitude from RCS (arbitrary scaling for ADC range)."""
        # Simple model: amplitude proportional to sqrt(RCS) / R^2
        # Normalized so 0 dBsm at 100m gives roughly 50% ADC scale
        rcs_linear = 10 ** (self.rcs_dbsm / 10.0)
        if self.range_m <= 0:
            return 0.0
        amp = math.sqrt(rcs_linear) / (self.range_m**2)
        # Scale to ADC range: 100m/0dBsm -> ~64 counts (half of 128 peak-to-peak)
        return amp * (100.0**2) * 64.0

    def __repr__(self):
        return (
            f"Target(range={self.range_m:.1f}m, vel={self.velocity_mps:.1f}m/s, "
            f"RCS={self.rcs_dbsm:.1f}dBsm, delay={self.delay_samples:.1f}samp)"
        )


# =============================================================================
# IF chirp signal generation
# =============================================================================


def generate_if_chirp(n_samples, chirp_bw=CHIRP_BW, f_if=F_IF, fs=FS_ADC):
    """
    Generate an IF chirp signal (the transmitted waveform as seen at IF).

    This models the PLFM chirp as a linear frequency sweep around the IF.
    The ADC sees this chirp after mixing with the LO.

    Args:
        n_samples: number of samples to generate
        chirp_bw: chirp bandwidth in Hz
        f_if: IF center frequency in Hz
        fs: sample rate in Hz

    Returns:
        (chirp_i, chirp_q): lists of float I/Q samples (normalized to [-1, 1])
    """
    chirp_i = []
    chirp_q = []
    chirp_rate = chirp_bw / (n_samples / fs)  # Hz/s

    for n in range(n_samples):
        t = n / fs
        # Instantaneous frequency: f_if - chirp_bw/2 + chirp_rate * t
        # Phase: integral of 2*pi*f(t)*dt
        _f_inst = f_if - chirp_bw / 2 + chirp_rate * t  # noqa: F841 — documents instantaneous frequency formula
        phase = 2 * math.pi * (f_if - chirp_bw / 2) * t + math.pi * chirp_rate * t * t
        chirp_i.append(math.cos(phase))
        chirp_q.append(math.sin(phase))

    return chirp_i, chirp_q


def generate_reference_chirp_q15(n_fft=FFT_SIZE, chirp_bw=CHIRP_BW, f_if=F_IF, fs=FS_ADC):
    """
    Generate a reference chirp in Q15 format for the matched filter.

    The reference chirp is the expected received signal (zero-delay, zero-Doppler).
    Padded with zeros to FFT_SIZE.

    Returns:
        (ref_re, ref_im): lists of N_FFT signed 16-bit integers
    """
    # Generate chirp for a reasonable number of samples
    # The chirp duration determines how many samples of the reference are non-zero
    # For 30 us chirp at 100 MHz (after decimation): 3000 samples
    # But FFT is 1024, so we use 1024 samples of the chirp
    chirp_samples = min(n_fft, int(T_LONG_CHIRP * FS_SYS))

    ref_re = [0] * n_fft
    ref_im = [0] * n_fft

    chirp_rate = chirp_bw / T_LONG_CHIRP

    for n in range(chirp_samples):
        t = n / FS_SYS
        # After DDC, the chirp is at baseband
        # The beat frequency from a target at delay tau is: f_beat = chirp_rate * tau
        # Reference chirp is the TX chirp at baseband (zero delay)
        phase = math.pi * chirp_rate * t * t
        re_val = int(round(32767 * 0.9 * math.cos(phase)))
        im_val = int(round(32767 * 0.9 * math.sin(phase)))
        ref_re[n] = max(-32768, min(32767, re_val))
        ref_im[n] = max(-32768, min(32767, im_val))

    return ref_re, ref_im


# =============================================================================
# ADC sample generation with targets
# =============================================================================


def generate_adc_samples(targets, n_samples, noise_stddev=3.0, clutter_amplitude=0.0, seed=42):
    """
    Generate synthetic ADC samples for a radar scene.

    Models:
      - Multiple targets at different ranges (delays)
      - Each target produces a delayed, attenuated copy of the TX chirp at IF
      - Doppler shift applied as phase rotation
      - Additive white Gaussian noise
      - Optional clutter

    Args:
        targets: list of Target objects
        n_samples: number of ADC samples at 400 MSPS
        noise_stddev: noise standard deviation in ADC LSBs
        clutter_amplitude: clutter amplitude in ADC LSBs
        seed: random seed for reproducibility

    Returns:
        list of n_samples 8-bit unsigned integers (0-255)
    """
    # Simple LCG random number generator (no numpy dependency)
    rng_state = seed

    def next_rand():
        nonlocal rng_state
        rng_state = (rng_state * 1103515245 + 12345) & 0x7FFFFFFF
        return rng_state

    def rand_gaussian():
        """Box-Muller transform using LCG."""
        while True:
            u1 = next_rand() / 0x7FFFFFFF
            u2 = next_rand() / 0x7FFFFFFF
            if u1 > 1e-10:
                break
        return math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)

    # Generate TX chirp (at IF) - this is what the ADC would see from a target
    chirp_rate = CHIRP_BW / T_LONG_CHIRP
    chirp_samples = int(T_LONG_CHIRP * FS_ADC)  # 12000 samples at 400 MSPS

    adc_float = [0.0] * n_samples

    for target in targets:
        delay_samp = target.delay_samples
        amp = target.amplitude
        doppler_hz = target.doppler_hz
        phase0 = target.phase_deg * math.pi / 180.0

        for n in range(n_samples):
            # Check if this sample falls within the delayed chirp
            n_delayed = n - delay_samp
            if n_delayed < 0 or n_delayed >= chirp_samples:
                continue

            t = n / FS_ADC
            t_delayed = n_delayed / FS_ADC

            # Signal at IF: cos(2*pi*f_if*t + pi*chirp_rate*t_delayed^2 + doppler + phase)
            phase = (
                2 * math.pi * F_IF * t
                + math.pi * chirp_rate * t_delayed * t_delayed
                + 2 * math.pi * doppler_hz * t
                + phase0
            )

            adc_float[n] += amp * math.cos(phase)

    # Add noise
    for n in range(n_samples):
        adc_float[n] += noise_stddev * rand_gaussian()

    # Add clutter (slow-varying, correlated noise)
    if clutter_amplitude > 0:
        clutter_phase = 0.0
        clutter_freq = 0.001  # Very slow variation
        for n in range(n_samples):
            clutter_phase += 2 * math.pi * clutter_freq
            adc_float[n] += clutter_amplitude * math.sin(clutter_phase + rand_gaussian() * 0.1)

    # Quantize to 8-bit unsigned (0-255), centered at 128
    adc_samples = []
    for val in adc_float:
        quantized = int(round(val + 128))
        quantized = max(0, min(255, quantized))
        adc_samples.append(quantized)

    return adc_samples


def generate_baseband_samples(targets, n_samples_baseband, noise_stddev=0.5, seed=42):
    """
    Generate synthetic baseband I/Q samples AFTER DDC.

    This bypasses the DDC entirely, generating what the DDC output should look
    like for given targets. Useful for testing matched filter and downstream
    processing without running through NCO/mixer/CIC/FIR.

    Each target produces a beat frequency: f_beat = chirp_rate * delay
    After DDC, the signal is at baseband with this beat frequency.

    Args:
        targets: list of Target objects
        n_samples_baseband: number of baseband samples (at 100 MHz)
        noise_stddev: noise in Q15 LSBs
        seed: random seed

    Returns:
        (bb_i, bb_q): lists of signed 16-bit integers (Q15)
    """
    rng_state = seed

    def next_rand():
        nonlocal rng_state
        rng_state = (rng_state * 1103515245 + 12345) & 0x7FFFFFFF
        return rng_state

    def rand_gaussian():
        while True:
            u1 = next_rand() / 0x7FFFFFFF
            u2 = next_rand() / 0x7FFFFFFF
            if u1 > 1e-10:
                break
        return math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)

    chirp_rate = CHIRP_BW / T_LONG_CHIRP
    bb_i_float = [0.0] * n_samples_baseband
    bb_q_float = [0.0] * n_samples_baseband

    for target in targets:
        f_beat = chirp_rate * target.delay_s  # Beat frequency
        amp = target.amplitude / 4.0  # Scale down for baseband (DDC gain ~ 1/4)
        doppler_hz = target.doppler_hz
        phase0 = target.phase_deg * math.pi / 180.0

        for n in range(n_samples_baseband):
            t = n / FS_SYS
            phase = 2 * math.pi * (f_beat + doppler_hz) * t + phase0
            bb_i_float[n] += amp * math.cos(phase)
            bb_q_float[n] += amp * math.sin(phase)

    # Add noise and quantize to Q15
    bb_i = []
    bb_q = []
    for n in range(n_samples_baseband):
        i_val = int(round(bb_i_float[n] + noise_stddev * rand_gaussian()))
        q_val = int(round(bb_q_float[n] + noise_stddev * rand_gaussian()))
        bb_i.append(max(-32768, min(32767, i_val)))
        bb_q.append(max(-32768, min(32767, q_val)))

    return bb_i, bb_q


# =============================================================================
# Multi-chirp frame generation (for Doppler processing)
# =============================================================================


def generate_doppler_frame(
    targets, n_chirps=CHIRPS_PER_FRAME, n_range_bins=RANGE_BINS, noise_stddev=0.5, seed=42
):
    """
    Generate a complete Doppler frame (32 chirps x 64 range bins).

    Each chirp sees a phase rotation due to target velocity:
      phase_shift_per_chirp = 2*pi * doppler_hz * T_chirp_repeat

    Args:
        targets: list of Target objects
        n_chirps: chirps per frame (32)
        n_range_bins: range bins per chirp (64)

    Returns:
        (frame_i, frame_q): [n_chirps][n_range_bins] arrays of signed 16-bit
    """
    rng_state = seed

    def next_rand():
        nonlocal rng_state
        rng_state = (rng_state * 1103515245 + 12345) & 0x7FFFFFFF
        return rng_state

    def rand_gaussian():
        while True:
            u1 = next_rand() / 0x7FFFFFFF
            u2 = next_rand() / 0x7FFFFFFF
            if u1 > 1e-10:
                break
        return math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)

    frame_i = []
    frame_q = []

    for chirp_idx in range(n_chirps):
        chirp_i = [0.0] * n_range_bins
        chirp_q = [0.0] * n_range_bins

        for target in targets:
            # Which range bin does this target fall in?
            # After matched filter + range decimation:
            # range_bin = target_delay_in_baseband_samples / decimation_factor
            delay_baseband_samples = target.delay_s * FS_SYS
            range_bin_float = delay_baseband_samples * n_range_bins / FFT_SIZE
            range_bin = int(round(range_bin_float))

            if range_bin < 0 or range_bin >= n_range_bins:
                continue

            # Amplitude (simplified)
            amp = target.amplitude / 4.0

            # Doppler phase for this chirp.
            # The frame uses staggered PRF: chirps 0-15 use the long PRI,
            # chirps 16-31 use the short PRI.
            if chirp_idx < CHIRPS_PER_SUBFRAME:
                slow_time_s = chirp_idx * T_PRI_LONG
            else:
                slow_time_s = (CHIRPS_PER_SUBFRAME * T_PRI_LONG) + (
                    (chirp_idx - CHIRPS_PER_SUBFRAME) * T_PRI_SHORT
                )

            doppler_phase = 2 * math.pi * target.doppler_hz * slow_time_s
            total_phase = doppler_phase + target.phase_deg * math.pi / 180.0

            # Spread across a few bins (sinc-like response from matched filter)
            for delta in range(-2, 3):
                rb = range_bin + delta
                if 0 <= rb < n_range_bins:
                    # sinc-like weighting
                    if delta == 0:
                        weight = 1.0
                    else:
                        weight = 0.2 / abs(delta)
                    chirp_i[rb] += amp * weight * math.cos(total_phase)
                    chirp_q[rb] += amp * weight * math.sin(total_phase)

        # Add noise and quantize
        row_i = []
        row_q = []
        for rb in range(n_range_bins):
            i_val = int(round(chirp_i[rb] + noise_stddev * rand_gaussian()))
            q_val = int(round(chirp_q[rb] + noise_stddev * rand_gaussian()))
            row_i.append(max(-32768, min(32767, i_val)))
            row_q.append(max(-32768, min(32767, q_val)))

        frame_i.append(row_i)
        frame_q.append(row_q)

    return frame_i, frame_q


# =============================================================================
# Output file generators
# =============================================================================


def write_hex_file(filepath, samples, bits=8):
    """
    Write samples to hex file for Verilog $readmemh.

    Args:
        filepath: output file path
        samples: list of integer samples
        bits: bit width per sample (8 for ADC, 16 for baseband)
    """
    hex_digits = (bits + 3) // 4
    fmt = f"{{:0{hex_digits}X}}"

    with open(filepath, "w") as f:
        f.write(f"// {len(samples)} samples, {bits}-bit, hex format for $readmemh\n")
        for i, s in enumerate(samples):
            if bits <= 8:
                val = s & 0xFF
            elif bits <= 16:
                val = s & 0xFFFF
            elif bits <= 32:
                val = s & 0xFFFFFFFF
            else:
                val = s & ((1 << bits) - 1)
            f.write(fmt.format(val) + "\n")

    print(f"  Wrote {len(samples)} samples to {filepath}")


def write_csv_file(filepath, columns, headers=None):
    """
    Write multi-column data to CSV.

    Args:
        filepath: output file path
        columns: list of lists (each list is a column)
        headers: list of column header strings
    """
    n_rows = len(columns[0])
    with open(filepath, "w") as f:
        if headers:
            f.write(",".join(headers) + "\n")
        for i in range(n_rows):
            row = [str(col[i]) for col in columns]
            f.write(",".join(row) + "\n")

    print(f"  Wrote {n_rows} rows to {filepath}")


# =============================================================================
# Pre-built test scenarios
# =============================================================================


def scenario_single_target(range_m=500, velocity=0, rcs=0, n_adc_samples=16384):
    """
    Single stationary target at specified range.
    Good for validating matched filter range response.
    """
    target = Target(range_m=range_m, velocity_mps=velocity, rcs_dbsm=rcs)
    print(f"Scenario: Single target at {range_m}m")
    print(f"  {target}")
    print(f"  Beat freq: {CHIRP_BW / T_LONG_CHIRP * target.delay_s:.0f} Hz")
    print(f"  Delay: {target.delay_samples:.1f} ADC samples")

    adc = generate_adc_samples([target], n_adc_samples, noise_stddev=2.0)
    return adc, [target]


def scenario_two_targets(n_adc_samples=16384):
    """
    Two targets at different ranges — tests range resolution.
    Separation: ~2x range resolution (15m).
    """
    targets = [
        Target(range_m=300, velocity_mps=0, rcs_dbsm=10, phase_deg=0),
        Target(range_m=315, velocity_mps=0, rcs_dbsm=10, phase_deg=45),
    ]
    print("Scenario: Two targets (range resolution test)")
    for t in targets:
        print(f"  {t}")

    adc = generate_adc_samples(targets, n_adc_samples, noise_stddev=2.0)
    return adc, targets


def scenario_multi_target(n_adc_samples=16384):
    """
    Five targets at various ranges and velocities — comprehensive test.
    """
    targets = [
        Target(range_m=100, velocity_mps=0, rcs_dbsm=20, phase_deg=0),
        Target(range_m=500, velocity_mps=30, rcs_dbsm=10, phase_deg=90),
        Target(range_m=1000, velocity_mps=-15, rcs_dbsm=5, phase_deg=180),
        Target(range_m=2000, velocity_mps=50, rcs_dbsm=0, phase_deg=45),
        Target(range_m=5000, velocity_mps=-5, rcs_dbsm=-5, phase_deg=270),
    ]
    print("Scenario: Multi-target (5 targets)")
    for t in targets:
        print(f"  {t}")

    adc = generate_adc_samples(targets, n_adc_samples, noise_stddev=3.0)
    return adc, targets


def scenario_noise_only(n_adc_samples=16384, noise_stddev=5.0):
    """
    Noise-only scene — baseline for false alarm characterization.
    """
    print(f"Scenario: Noise only (stddev={noise_stddev})")
    adc = generate_adc_samples([], n_adc_samples, noise_stddev=noise_stddev)
    return adc, []


def scenario_dc_tone(n_adc_samples=16384, adc_value=128):
    """
    DC input — validates CIC decimation and DC response.
    """
    print(f"Scenario: DC tone (ADC value={adc_value})")
    return [adc_value] * n_adc_samples, []


def scenario_sine_wave(n_adc_samples=16384, freq_hz=1e6, amplitude=50):
    """
    Pure sine wave at ADC input — validates NCO/mixer frequency response.
    """
    print(f"Scenario: Sine wave at {freq_hz / 1e6:.1f} MHz, amplitude={amplitude}")
    adc = []
    for n in range(n_adc_samples):
        t = n / FS_ADC
        val = int(round(128 + amplitude * math.sin(2 * math.pi * freq_hz * t)))
        adc.append(max(0, min(255, val)))
    return adc, []


# =============================================================================
# Main: Generate all test vectors
# =============================================================================


def generate_all_test_vectors(output_dir=None):
    """
    Generate a complete set of test vectors for co-simulation.

    Creates:
      - adc_single_target.hex: ADC samples for single target
      - adc_multi_target.hex: ADC samples for 5 targets
      - adc_noise_only.hex: Noise-only ADC samples
      - adc_dc.hex: DC input
      - adc_sine_1mhz.hex: 1 MHz sine wave
      - ref_chirp_i.hex / ref_chirp_q.hex: Reference chirp for matched filter
      - bb_single_target_i.hex / _q.hex: Baseband I/Q for matched filter test
      - scenario_info.csv: Target parameters for each scenario
    """
    if output_dir is None:
        output_dir = os.path.dirname(os.path.abspath(__file__))

    print("=" * 60)
    print("Generating AERIS-10 Test Vectors")
    print(f"Output directory: {output_dir}")
    print("=" * 60)

    n_adc = 16384  # ~41 us of ADC data

    # --- Scenario 1: Single target ---
    print("\n--- Scenario 1: Single Target ---")
    adc1, targets1 = scenario_single_target(range_m=500, n_adc_samples=n_adc)
    write_hex_file(os.path.join(output_dir, "adc_single_target.hex"), adc1, bits=8)

    # --- Scenario 2: Multi-target ---
    print("\n--- Scenario 2: Multi-Target ---")
    adc2, targets2 = scenario_multi_target(n_adc_samples=n_adc)
    write_hex_file(os.path.join(output_dir, "adc_multi_target.hex"), adc2, bits=8)

    # --- Scenario 3: Noise only ---
    print("\n--- Scenario 3: Noise Only ---")
    adc3, _ = scenario_noise_only(n_adc_samples=n_adc)
    write_hex_file(os.path.join(output_dir, "adc_noise_only.hex"), adc3, bits=8)

    # --- Scenario 4: DC ---
    print("\n--- Scenario 4: DC Input ---")
    adc4, _ = scenario_dc_tone(n_adc_samples=n_adc)
    write_hex_file(os.path.join(output_dir, "adc_dc.hex"), adc4, bits=8)

    # --- Scenario 5: Sine wave ---
    print("\n--- Scenario 5: 1 MHz Sine ---")
    adc5, _ = scenario_sine_wave(n_adc_samples=n_adc, freq_hz=1e6, amplitude=50)
    write_hex_file(os.path.join(output_dir, "adc_sine_1mhz.hex"), adc5, bits=8)

    # --- Reference chirp for matched filter ---
    print("\n--- Reference Chirp ---")
    ref_re, ref_im = generate_reference_chirp_q15()
    write_hex_file(os.path.join(output_dir, "ref_chirp_i.hex"), ref_re, bits=16)
    write_hex_file(os.path.join(output_dir, "ref_chirp_q.hex"), ref_im, bits=16)

    # --- Baseband samples for matched filter test (bypass DDC) ---
    print("\n--- Baseband Samples (bypass DDC) ---")
    bb_targets = [
        Target(range_m=500, velocity_mps=0, rcs_dbsm=10),
        Target(range_m=1500, velocity_mps=20, rcs_dbsm=5),
    ]
    bb_i, bb_q = generate_baseband_samples(bb_targets, FFT_SIZE, noise_stddev=1.0)
    write_hex_file(os.path.join(output_dir, "bb_mf_test_i.hex"), bb_i, bits=16)
    write_hex_file(os.path.join(output_dir, "bb_mf_test_q.hex"), bb_q, bits=16)

    # --- Scenario info CSV ---
    print("\n--- Scenario Info ---")
    with open(os.path.join(output_dir, "scenario_info.txt"), "w") as f:
        f.write("AERIS-10 Test Vector Scenarios\n")
        f.write("=" * 60 + "\n\n")

        f.write("System Parameters:\n")
        f.write(f"  Carrier: {F_CARRIER / 1e9:.1f} GHz\n")
        f.write(f"  IF: {F_IF / 1e6:.0f} MHz\n")
        f.write(f"  Chirp BW: {CHIRP_BW / 1e6:.0f} MHz\n")
        f.write(f"  ADC: {FS_ADC / 1e6:.0f} MSPS, {ADC_BITS}-bit\n")
        f.write(f"  Range resolution: {RANGE_RESOLUTION:.1f} m\n")
        f.write(f"  Wavelength: {WAVELENGTH * 1000:.2f} mm\n")
        f.write("\n")

        f.write("Scenario 1: Single target\n")
        for t in targets1:
            f.write(f"  {t}\n")

        f.write("\nScenario 2: Multi-target (5 targets)\n")
        for t in targets2:
            f.write(f"  {t}\n")

        f.write("\nScenario 3: Noise only (stddev=5.0 LSB)\n")
        f.write("\nScenario 4: DC input (value=128)\n")
        f.write("\nScenario 5: 1 MHz sine wave (amplitude=50 LSB)\n")

        f.write("\nBaseband MF test targets:\n")
        for t in bb_targets:
            f.write(f"  {t}\n")

    print(f"\n  Wrote scenario info to {os.path.join(output_dir, 'scenario_info.txt')}")

    print("\n" + "=" * 60)
    print("ALL TEST VECTORS GENERATED")
    print("=" * 60)

    return {
        "adc_single": adc1,
        "adc_multi": adc2,
        "adc_noise": adc3,
        "adc_dc": adc4,
        "adc_sine": adc5,
        "ref_chirp_re": ref_re,
        "ref_chirp_im": ref_im,
        "bb_i": bb_i,
        "bb_q": bb_q,
    }


if __name__ == "__main__":
    generate_all_test_vectors()
