#!/usr/bin/env python3
"""
golden_reference.py — AERIS-10 FPGA bit-accurate golden reference model

Uses ADI CN0566 Phaser radar data (10.525 GHz X-band FMCW) to validate
the FPGA signal processing pipeline stage by stage:

    ADC → DDC (NCO+mixer+CIC+FIR) → Range FFT → Doppler FFT → Detection

The model replicates exact RTL fixed-point arithmetic (bit widths, truncation,
rounding, saturation) so outputs can be compared bit-for-bit against Icarus
Verilog simulation results.

Generates .hex stimulus files for RTL testbenches and .npy reference files
for comparison.

Usage:
    python3 golden_reference.py [--frame N] [--plot]
"""

import numpy as np
import os
import argparse

# ===========================================================================
# Configuration — exact match to RTL parameters
# ===========================================================================

# ADC
ADC_BITS = 8                    # ad9484: 8-bit unsigned

# NCO
NCO_PHASE_WIDTH = 32
NCO_PHASE_INC = 0x4CCCCCCD     # 120 MHz IF at 400 MHz Fs
NCO_LUT_SIZE = 64               # Quarter-wave sine LUT entries
NCO_OUT_BITS = 16                # Signed 16-bit sin/cos output

# Mixer
MIXER_IN_BITS = 18               # ADC sign-extended to 18-bit
MIXER_PRODUCT_BITS = 34          # 18 x 16 = 34
MIXER_TRUNCATE_SHIFT = 16        # mixed_i[33:16] fed to CIC

# CIC
CIC_STAGES = 5
CIC_DECIMATION = 4
CIC_DIFFERENTIAL_DELAY = 1
CIC_ACC_WIDTH = 48
CIC_COMB_WIDTH = 28
CIC_GAIN_SHIFT = 10              # >>> 10 to normalize 4^5 = 1024
CIC_OUT_BITS = 18                # Saturated to signed 18-bit

# FIR
FIR_TAPS = 32
FIR_COEFF_WIDTH = 18
FIR_DATA_WIDTH = 18
FIR_ACCUM_WIDTH = 36
# Coefficients from fir_lowpass.v (18-bit signed hex, 18'sh notation)
FIR_COEFFS_HEX = [
    0x000AD, 0x000CE, 0x3FD87, 0x002A6,
    0x000E0, 0x3F8C0, 0x00A45, 0x3FD82,
    0x3F0B5, 0x01CAD, 0x3EE59, 0x3E821,
    0x04841, 0x3B340, 0x3E299, 0x1FFFF,
    0x1FFFF, 0x3E299, 0x3B340, 0x04841,
    0x3E821, 0x3EE59, 0x01CAD, 0x3F0B5,
    0x3FD82, 0x00A45, 0x3F8C0, 0x000E0,
    0x002A6, 0x3FD87, 0x000CE, 0x000AD,
]

# DDC output interface
DDC_OUT_BITS = 16                # 18 → 16 bit with rounding + saturation

FFT_SIZE = 1024
FFT_DATA_W = 16
FFT_INTERNAL_W = 32
FFT_TWIDDLE_W = 16

# Doppler — dual 16-pt FFT architecture
DOPPLER_FFT_SIZE = 16            # per sub-frame
DOPPLER_TOTAL_BINS = 32          # total output (2 sub-frames x 16)
DOPPLER_RANGE_BINS = 64
DOPPLER_CHIRPS = 32
CHIRPS_PER_SUBFRAME = 16
DOPPLER_WINDOW_TYPE = 0          # Hamming

# 16-point Hamming window coefficients from doppler_processor.v (Q15)
HAMMING_Q15 = [
    0x0A3D, 0x0E5C, 0x1B6D, 0x3088,
    0x4B33, 0x6573, 0x7642, 0x7F62,
    0x7F62, 0x7642, 0x6573, 0x4B33,
    0x3088, 0x1B6D, 0x0E5C, 0x0A3D,
]

# ADI dataset parameters
ADI_SAMPLE_RATE = 4e6            # 4 MSPS
ADI_IF_FREQ = 100e3             # 100 kHz IF
ADI_RF_FREQ = 9.9e9             # 9.9 GHz
ADI_CHIRP_BW = 500e6            # 500 MHz
ADI_RAMP_TIME = 300e-6          # 300 us
ADI_NUM_CHIRPS = 256
ADI_SAMPLES_PER_CHIRP = 1079

# AERIS-10 parameters
AERIS_FS = 400e6                 # 400 MHz ADC clock
AERIS_IF = 120e6                 # 120 MHz IF


# ===========================================================================
# Helper: Convert hex to signed integer
# ===========================================================================
def hex_to_signed(val, bits):
    """Convert unsigned hex value to signed integer."""
    if val >= (1 << (bits - 1)):
        return val - (1 << bits)
    return val


def signed_to_hex(val, bits):
    """Convert signed integer to hex string (no prefix)."""
    if val < 0:
        val = val + (1 << bits)
    return format(val & ((1 << bits) - 1), f'0{(bits + 3) // 4}X')


def saturate(val, bits):
    """Saturate signed value to fit in 'bits' width."""
    max_pos = (1 << (bits - 1)) - 1
    max_neg = -(1 << (bits - 1))
    if val > max_pos:
        return max_pos
    if val < max_neg:
        return max_neg
    return int(val)


# ===========================================================================
# Stage 0: Load ADI data and requantize to 8-bit ADC
# ===========================================================================
def load_and_quantize_adi_data(data_path, config_path, frame_idx=0):
    """
    Load ADI Phaser radar data and requantize to 8-bit unsigned ADC format.
    
    The ADI data is complex IQ at baseband. AERIS-10 has a real 8-bit ADC
    with a 120 MHz IF. We need to:
    1. Take one frame of 256 chirps x 1079 samples
    2. Use only 32 chirps (matching AERIS-10 CHIRPS_PER_FRAME)
    3. Truncate to 1024 samples (matching FFT_SIZE)
    4. Upconvert to 120 MHz IF (add I*cos - Q*sin) to create real signal
    5. Quantize to 8-bit unsigned (matching AD9484)
    """
    data = np.load(data_path, allow_pickle=True)
    config = np.load(config_path, allow_pickle=True)
    
    
    # Extract one frame
    frame = data[frame_idx]  # (256, 1079) complex
    
    # Use first 32 chirps, first 1024 samples
    iq_block = frame[:DOPPLER_CHIRPS, :FFT_SIZE]  # (32, 1024) complex
    
    # The ADI data is baseband complex IQ at 4 MSPS.
    # AERIS-10 sees a real signal at 400 MSPS with 120 MHz IF.
    # To create a realistic ADC stimulus, we upconvert to IF:
    #   x_real(n) = Re{IQ(n)} * cos(2*pi*f_IF*n/Fs) - Im{IQ(n)} * sin(2*pi*f_IF*n/Fs)
    #
    # However, the ADI data at 4 MSPS doesn't have the bandwidth to fill
    # 400 MSPS. Instead, for DDC validation we create a simpler approach:
    # feed the baseband IQ directly into the post-DDC stage, bypassing
    # the NCO/mixer/CIC. This is actually MORE useful because:
    # - DDC is already validated by existing cosim tests (tb_ddc_cosim.v)
    # - What we REALLY want to test is FFT + Doppler + detection with real data
    # - We can still validate DDC bit-accuracy separately
    
    # Scale IQ data to realistic DDC output level.
    # The 1024-point FFT has no output /N scaling (forward mode), so the
    # processing gain can be up to ~300x for coherent signals.  To keep
    # the 16-bit output from saturating on most bins (which destroys
    # dynamic range and makes SNR comparison meaningless), we scale the
    # input so the peak magnitude is ~200 — representative of a moderate
    # radar return through the DDC chain (-40 dB below full-scale ADC).
    # At this level, < 0.01% of FFT output bins saturate.
    INPUT_PEAK_TARGET = 200
    max_abs = np.max(np.abs(iq_block))
    scale = INPUT_PEAK_TARGET / max_abs
    
    iq_scaled = iq_block * scale
    iq_i = np.round(np.real(iq_scaled)).astype(np.int64)
    iq_q = np.round(np.imag(iq_scaled)).astype(np.int64)
    
    # Clamp to 16-bit signed
    iq_i = np.clip(iq_i, -32768, 32767)
    iq_q = np.clip(iq_q, -32768, 32767)
    
    
    # Also create 8-bit ADC stimulus for DDC validation
    # Use just one chirp of real-valued data (I channel only, shifted to unsigned)
    chirp0_real = np.real(frame[0, :FFT_SIZE])
    chirp0_norm = chirp0_real / np.max(np.abs(chirp0_real))
    adc_8bit = np.round(chirp0_norm * 127 + 128).astype(np.uint8)
    adc_8bit = np.clip(adc_8bit, 0, 255)
    
    return iq_i, iq_q, adc_8bit, config


# ===========================================================================
# Stage 1: NCO Model (bit-accurate)
# ===========================================================================
def build_nco_lut():
    """Build the exact quarter-wave sine LUT from nco_400m_enhanced.v."""
    lut = np.zeros(64, dtype=np.int32)
    # Values from nco_400m_enhanced.v sin_lut initialization
    vals = [
        0x0000, 0x0324, 0x0648, 0x096A, 0x0C8C, 0x0FAB, 0x12C8, 0x15E2,
        0x18F9, 0x1C0B, 0x1F1A, 0x2223, 0x2528, 0x2826, 0x2B1F, 0x2E11,
        0x30FB, 0x33DF, 0x36BA, 0x398C, 0x3C56, 0x3F17, 0x41CE, 0x447A,
        0x471C, 0x49B4, 0x4C3F, 0x4EBF, 0x5133, 0x539B, 0x55F5, 0x5842,
        0x5A82, 0x5CB3, 0x5ED7, 0x60EB, 0x62F1, 0x64E8, 0x66CF, 0x68A6,
        0x6A6D, 0x6C23, 0x6DC9, 0x6F5E, 0x70E2, 0x7254, 0x73B5, 0x7504,
        0x7641, 0x776B, 0x7884, 0x7989, 0x7A7C, 0x7B5C, 0x7C29, 0x7CE3,
        0x7D89, 0x7E1D, 0x7E9C, 0x7F09, 0x7F61, 0x7FA6, 0x7FD8, 0x7FF5,
    ]
    for i, v in enumerate(vals):
        lut[i] = v
    return lut


def nco_lookup(phase_accum, sin_lut):
    """
    Replicate RTL NCO quarter-wave lookup with quadrant sign selection.
    Input: 32-bit phase accumulator value
    Output: (sin_out, cos_out) as signed 16-bit integers
    """
    lut_address = (phase_accum >> 24) & 0xFF  # top 8 bits
    quadrant = (lut_address >> 6) & 0x3
    
    # Mirror index for odd quadrants
    lut_idx = ~lut_address & 63 if quadrant & 1 ^ quadrant >> 1 & 1 else lut_address & 63
    
    sin_abs = int(sin_lut[lut_idx])
    cos_abs = int(sin_lut[63 - lut_idx])
    
    # Quadrant sign application
    if quadrant == 0:    # Q I: sin+, cos+
        sin_out = sin_abs
        cos_out = cos_abs
    elif quadrant == 1:  # Q II: sin+, cos-
        sin_out = sin_abs
        cos_out = -cos_abs
    elif quadrant == 2:  # Q III: sin-, cos-
        sin_out = -sin_abs
        cos_out = -cos_abs
    else:                # Q IV: sin-, cos+
        sin_out = -sin_abs
        cos_out = cos_abs
    
    # Clamp to signed 16-bit
    sin_out = saturate(sin_out, 16)
    cos_out = saturate(cos_out, 16)
    
    return sin_out, cos_out


# ===========================================================================
# Stage 1: DDC Model (NCO + Mixer + CIC + FIR + Interface)
# ===========================================================================
def run_ddc(adc_samples):
    """
    Bit-accurate DDC model. Takes 8-bit unsigned ADC samples at 400 MHz,
    returns 16-bit signed I/Q baseband at 100 MHz.
    
    Pipeline:
    1. ADC sign conversion: 8-bit unsigned → 18-bit signed
    2. NCO: 32-bit phase accumulator → 16-bit sin/cos via quarter-wave LUT
    3. Mixer: 18-bit * 16-bit = 34-bit, truncate [33:16] → 18-bit CIC input
    4. CIC: 5-stage, decimate-by-4, normalize >>>10, saturate to 18-bit
    5. FIR: 32-tap, 18-bit in/out, 36-bit accumulator
    6. Interface: 18-bit → 16-bit with convergent rounding + saturation
    """
    n_samples = len(adc_samples)
    sin_lut = build_nco_lut()
    
    # Build FIR coefficients as signed integers
    fir_coeffs = np.array([hex_to_signed(c, 18) for c in FIR_COEFFS_HEX], dtype=np.int64)
    
    
    # --- NCO + Mixer ---
    phase_accum = np.int64(0)
    mixed_i = np.zeros(n_samples, dtype=np.int64)
    mixed_q = np.zeros(n_samples, dtype=np.int64)
    
    for n in range(n_samples):
        # ADC sign conversion: RTL does offset binary → signed 18-bit
        # adc_signed_w = {1'b0, adc_data, 9'b0} - {1'b0, 8'hFF, 9'b0}/2
        # Simplified: center around zero, scale to 18-bit
        adc_val = int(adc_samples[n])
        adc_signed = (adc_val << 9) - 0xFF00  # Exact RTL: {1'b0,adc,9'b0} - {1'b0,8'hFF,9'b0}/2
        adc_signed = saturate(adc_signed, 18)
        
        # NCO lookup (ignoring dithering for golden reference)
        sin_out, cos_out = nco_lookup(int(phase_accum), sin_lut)
        
        # Mixer: 18-bit x 16-bit = 34-bit product
        prod_i = adc_signed * cos_out  # I = ADC * cos
        prod_q = adc_signed * sin_out  # Q = ADC * sin
        
        # Truncate to 18-bit: [33:16] of 34-bit product
        mixed_i[n] = (prod_i >> 16) & 0x3FFFF
        if mixed_i[n] >= (1 << 17):
            mixed_i[n] -= (1 << 18)
        mixed_q[n] = (prod_q >> 16) & 0x3FFFF
        if mixed_q[n] >= (1 << 17):
            mixed_q[n] -= (1 << 18)
        
        # Phase accumulator update (ignore dithering for bit-accuracy)
        phase_accum = (phase_accum + NCO_PHASE_INC) & 0xFFFFFFFF
    
    
    # --- CIC Decimator (5-stage, decimate-by-4) ---
    # Integrator section (at 400 MHz rate)
    integrators = np.zeros((CIC_STAGES, n_samples + 1), dtype=np.int64)
    for n in range(n_samples):
        integrators[0][n + 1] = (integrators[0][n] + mixed_i[n]) & ((1 << CIC_ACC_WIDTH) - 1)
        for s in range(1, CIC_STAGES):
            integrators[s][n + 1] = (
                integrators[s][n] + integrators[s - 1][n + 1]
            ) & ((1 << CIC_ACC_WIDTH) - 1)
    
    # Downsample by 4
    n_decimated = n_samples // CIC_DECIMATION
    decimated = np.zeros(n_decimated, dtype=np.int64)
    for k in range(n_decimated):
        val = integrators[CIC_STAGES - 1][(k + 1) * CIC_DECIMATION]
        # Convert from unsigned modular to signed
        if val >= (1 << (CIC_ACC_WIDTH - 1)):
            val -= (1 << CIC_ACC_WIDTH)
        # Truncate to comb width
        decimated[k] = val & ((1 << CIC_COMB_WIDTH) - 1)
        if decimated[k] >= (1 << (CIC_COMB_WIDTH - 1)):
            decimated[k] -= (1 << CIC_COMB_WIDTH)
    
    # Comb section (at 100 MHz rate)
    comb = np.zeros((CIC_STAGES, n_decimated), dtype=np.int64)
    comb_delay = np.zeros(CIC_STAGES, dtype=np.int64)
    
    for k in range(n_decimated):
        # Stage 0
        comb[0][k] = decimated[k] - comb_delay[0]
        comb_delay[0] = decimated[k]
        # Stages 1-4
        for s in range(1, CIC_STAGES):
            comb[s][k] = comb[s - 1][k] - comb_delay[s]
            comb_delay[s] = comb[s - 1][k]
    
    # Gain normalization: >>> 10
    cic_output = np.zeros(n_decimated, dtype=np.int64)
    for k in range(n_decimated):
        scaled = comb[CIC_STAGES - 1][k] >> CIC_GAIN_SHIFT
        cic_output[k] = saturate(scaled, CIC_OUT_BITS)
    
    
    # --- FIR Filter (32-tap) ---
    delay_line = np.zeros(FIR_TAPS, dtype=np.int64)
    fir_output = np.zeros(n_decimated, dtype=np.int64)
    
    for k in range(n_decimated):
        # Shift delay line
        delay_line[1:] = delay_line[:-1]
        delay_line[0] = cic_output[k]
        
        # Compute FIR output
        accum = np.int64(0)
        for t in range(FIR_TAPS):
            prod = delay_line[t] * fir_coeffs[t]
            accum += prod
        
        # Output rounding: accumulator_reg[ACCUM_WIDTH-2:DATA_WIDTH-1] = [34:17]
        fir_output[k] = saturate((accum >> 17) & 0x3FFFF, 18)
        if fir_output[k] >= (1 << 17):
            fir_output[k] -= (1 << 18)
    
    
    # --- DDC Interface (18 → 16 bit) ---
    ddc_output = np.zeros(n_decimated, dtype=np.int64)
    for k in range(n_decimated):
        val = fir_output[k]
        trunc = (val >> 2) & 0xFFFF  # [17:2]
        if trunc >= (1 << 15):
            trunc -= (1 << 16)
        round_bit = (val >> 1) & 1
        
        # Saturation check
        if trunc == 32767 and round_bit:
            ddc_output[k] = 32767  # Saturate
        else:
            ddc_output[k] = saturate(trunc + round_bit, 16)
    
    
    return ddc_output


# ===========================================================================
# Stage 2: Range FFT (1024-point, bit-accurate)
# ===========================================================================
def load_twiddle_rom(twiddle_file):
    """Load the quarter-wave cosine ROM from .mem file."""
    rom = []
    with open(twiddle_file) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('//'):
                continue
            val = int(line, 16)
            if val >= (1 << 15):
                val -= (1 << 16)
            rom.append(val)
    return np.array(rom, dtype=np.int64)


def fft_twiddle_lookup(k, N, cos_rom):
    """Replicate RTL quarter-wave twiddle lookup."""
    N_QTR = N // 4
    N_HALF = N // 2
    
    k = k % N_HALF
    
    if k == 0:
        tw_cos = int(cos_rom[0])
        tw_sin = 0
    elif k == N_QTR:
        tw_cos = 0
        tw_sin = int(cos_rom[0])
    elif k < N_QTR:
        tw_cos = int(cos_rom[k])
        tw_sin = int(cos_rom[N_QTR - k])
    else:
        rom_idx = k - N_QTR
        tw_sin = int(cos_rom[rom_idx])
        rom_idx2 = N_HALF - k
        tw_cos = -int(cos_rom[rom_idx2])
    
    return tw_cos, tw_sin


def run_range_fft(iq_i, iq_q, twiddle_file=None):
    """
    Bit-accurate 1024-point radix-2 DIT FFT matching fft_engine.v.
    
    Input: 16-bit signed I/Q arrays (1024 samples)
    Output: 16-bit signed I/Q arrays (1024 bins, saturated from 32-bit internal)
    
    Matches RTL:
    - Bit-reversed input loading → sign-extended to 32-bit internal
    - 10 stages of radix-2 butterflies
    - Twiddle multiply: 32-bit * 16-bit = 48-bit, shift >>> 15
    - Add/subtract in 32-bit
    - Output: saturate 32-bit → 16-bit
    """
    N = FFT_SIZE
    LOG2N = int(np.log2(N))
    assert N == 1024 and LOG2N == 10
    
    # Load twiddle ROM
    if twiddle_file and os.path.exists(twiddle_file):
        cos_rom = load_twiddle_rom(twiddle_file)
    else:
        # Generate twiddle factors if file not available
        cos_rom = np.round(32767 * np.cos(2 * np.pi * np.arange(N // 4) / N)).astype(np.int64)
    
    
    # Bit-reverse and sign-extend to 32-bit internal width
    def bit_reverse(val, bits):
        result = 0
        for b in range(bits):
            if val & (1 << b):
                result |= (1 << (bits - 1 - b))
        return result
    
    mem_re = np.zeros(N, dtype=np.int64)
    mem_im = np.zeros(N, dtype=np.int64)
    
    for n in range(N):
        br = bit_reverse(n, LOG2N)
        # Sign-extend 16-bit to 32-bit
        mem_re[br] = int(iq_i[n])
        mem_im[br] = int(iq_q[n])
    
    # Butterfly computation: LOG2N stages
    half = 1
    for stg in range(LOG2N):
        for bfly in range(N // 2):
            idx = bfly & (half - 1)
            grp = bfly - idx
            addr_even = (grp << 1) | idx
            addr_odd = addr_even + half
            
            # Twiddle index via barrel shift
            tw_idx = (idx << (LOG2N - 1 - stg)) % (N // 2)
            tw_cos, tw_sin = fft_twiddle_lookup(tw_idx, N, cos_rom)
            
            # Read
            a_re = mem_re[addr_even]
            a_im = mem_im[addr_even]
            b_re = mem_re[addr_odd]
            b_im = mem_im[addr_odd]
            
            prod_re = b_re * tw_cos + b_im * tw_sin
            prod_im = b_im * tw_cos - b_re * tw_sin
            
            # Arithmetic right shift by (TWIDDLE_W - 1) = 15
            prod_re_shifted = prod_re >> 15
            prod_im_shifted = prod_im >> 15
            
            # Butterfly add/subtract
            mem_re[addr_even] = a_re + prod_re_shifted
            mem_im[addr_even] = a_im + prod_im_shifted
            mem_re[addr_odd] = a_re - prod_re_shifted
            mem_im[addr_odd] = a_im - prod_im_shifted
        
        half <<= 1
    
    # Output: saturate 32-bit to 16-bit
    out_re = np.zeros(N, dtype=np.int64)
    out_im = np.zeros(N, dtype=np.int64)
    for n in range(N):
        out_re[n] = saturate(mem_re[n], FFT_DATA_W)
        out_im[n] = saturate(mem_im[n], FFT_DATA_W)
    
    
    return out_re, out_im


# ===========================================================================
# Stage 2b: Range Bin Decimator (1024 → 64, bit-accurate)
# ===========================================================================
def run_range_bin_decimator(range_fft_i, range_fft_q,
                            mode=1, start_bin=0,
                            input_bins=1024, output_bins=64,
                            decimation_factor=16):
    """
    Bit-accurate model of range_bin_decimator.v (peak detection mode).

    Input:  range_fft_i/q — shape (N_chirps, 1024), 16-bit signed
    Output: decimated_i/q — shape (N_chirps, 64), 16-bit signed

    Modes:
        0 = simple decimation (take center sample of each group)
        1 = peak detection   (select max |I|+|Q| from each group of 16)
        2 = averaging        (sum group >> 4)

    RTL detail: abs_i = I[15] ? (~I + 1) : I   (unsigned 16-bit)
                cur_mag = {1'b0, abs_i} + {1'b0, abs_q}   (17-bit)
    For I = -32768 (0x8000): ~0x8000 + 1 = 0x8000 = 32768 unsigned → correct.
    """
    n_chirps = range_fft_i.shape[0]
    n_in     = range_fft_i.shape[1]
    assert n_in == input_bins, f"Expected {input_bins} input bins, got {n_in}"
    assert mode in (0, 1, 2), f"Invalid mode {mode}"

    decimated_i = np.zeros((n_chirps, output_bins), dtype=np.int64)
    decimated_q = np.zeros((n_chirps, output_bins), dtype=np.int64)


    for c in range(n_chirps):
        # Index into input, skip start_bin
        in_idx = start_bin

        for obin in range(output_bins):
            if mode == 1:
                # Peak detection: find max |I|+|Q| within group of decimation_factor
                best_i = 0
                best_q = 0
                best_mag = -1

                for s in range(decimation_factor):
                    if in_idx >= input_bins:
                        break
                    si = int(range_fft_i[c, in_idx])
                    sq = int(range_fft_q[c, in_idx])

                    # RTL absolute value on unsigned 16-bit wire:
                    # For signed input, interpret as 16-bit two's complement
                    # abs_i = I[15] ? (~I[15:0] + 1) : I[15:0]
                    # This naturally handles -32768 → 32768 (unsigned)
                    ai = (-si) & 0xFFFF if si < 0 else si & 0xFFFF
                    aq = (-sq) & 0xFFFF if sq < 0 else sq & 0xFFFF
                    mag = ai + aq  # 17-bit unsigned

                    if s == 0 or mag > best_mag:
                        best_i   = si
                        best_q   = sq
                        best_mag = mag

                    in_idx += 1

                decimated_i[c, obin] = best_i
                decimated_q[c, obin] = best_q

            elif mode == 0:
                # Simple decimation: take center sample (offset = decimation_factor/2)
                center = in_idx + decimation_factor // 2
                if center < input_bins:
                    decimated_i[c, obin] = int(range_fft_i[c, center])
                    decimated_q[c, obin] = int(range_fft_q[c, center])
                in_idx += decimation_factor

            elif mode == 2:
                # Averaging: sum group, then >> 4 (divide by 16)
                sum_i = np.int64(0)
                sum_q = np.int64(0)
                for _ in range(decimation_factor):
                    if in_idx >= input_bins:
                        break
                    sum_i += int(range_fft_i[c, in_idx])
                    sum_q += int(range_fft_q[c, in_idx])
                    in_idx += 1
                # RTL: sum_i[19:4], truncation (not rounding)
                decimated_i[c, obin] = int(sum_i) >> 4
                decimated_q[c, obin] = int(sum_q) >> 4


    return decimated_i, decimated_q


# ===========================================================================
# Stage 3: Doppler FFT (dual 16-point with Hamming window, bit-accurate)
# ===========================================================================
def run_doppler_fft(range_data_i, range_data_q, twiddle_file_16=None):
    """
    Bit-accurate Doppler processor matching doppler_processor.v (dual 16-pt FFT).

    Input: range_data_i/q shape (DOPPLER_CHIRPS, FFT_SIZE) — 16-bit signed
           Only first DOPPLER_RANGE_BINS columns are processed.
    Output: doppler_map_i/q shape (DOPPLER_RANGE_BINS, DOPPLER_TOTAL_BINS) — 16-bit signed

    Architecture per range bin:
      Sub-frame 0 (long PRI):  chirps 0..15  → 16-pt Hamming → 16-pt FFT → bins 0-15
      Sub-frame 1 (short PRI): chirps 16..31 → 16-pt Hamming → 16-pt FFT → bins 16-31
    """
    n_chirps = DOPPLER_CHIRPS
    n_range = DOPPLER_RANGE_BINS
    n_fft = DOPPLER_FFT_SIZE
    n_total = DOPPLER_TOTAL_BINS
    n_sf = CHIRPS_PER_SUBFRAME


    # Build 16-point Hamming window as signed 16-bit
    hamming = np.array([int(v) for v in HAMMING_Q15], dtype=np.int64)
    assert len(hamming) == n_fft, f"Hamming length {len(hamming)} != {n_fft}"

    # Build 16-point twiddle factors
    if twiddle_file_16 and os.path.exists(twiddle_file_16):
        cos_rom_16 = load_twiddle_rom(twiddle_file_16)
    else:
        cos_rom_16 = np.round(
            32767 * np.cos(2 * np.pi * np.arange(n_fft // 4) / n_fft)
        ).astype(np.int64)

    LOG2N_16 = 4
    doppler_map_i = np.zeros((n_range, n_total), dtype=np.int64)
    doppler_map_q = np.zeros((n_range, n_total), dtype=np.int64)

    for rbin in range(n_range):
        chirp_i = np.zeros(n_chirps, dtype=np.int64)
        chirp_q = np.zeros(n_chirps, dtype=np.int64)
        for c in range(n_chirps):
            chirp_i[c] = int(range_data_i[c, rbin])
            chirp_q[c] = int(range_data_q[c, rbin])

        # Process each sub-frame independently
        for sf in range(2):
            chirp_start = sf * n_sf
            bin_offset = sf * n_fft

            windowed_i = np.zeros(n_fft, dtype=np.int64)
            windowed_q = np.zeros(n_fft, dtype=np.int64)
            for k in range(n_fft):
                ci = chirp_i[chirp_start + k]
                cq = chirp_q[chirp_start + k]
                mult_i = ci * hamming[k]
                mult_q = cq * hamming[k]
                windowed_i[k] = saturate((mult_i + (1 << 14)) >> 15, 16)
                windowed_q[k] = saturate((mult_q + (1 << 14)) >> 15, 16)

            mem_re = np.zeros(n_fft, dtype=np.int64)
            mem_im = np.zeros(n_fft, dtype=np.int64)

            for n in range(n_fft):
                br = 0
                for b in range(LOG2N_16):
                    if n & (1 << b):
                        br |= (1 << (LOG2N_16 - 1 - b))
                mem_re[br] = windowed_i[n]
                mem_im[br] = windowed_q[n]

            half = 1
            for stg in range(LOG2N_16):
                for bfly in range(n_fft // 2):
                    idx = bfly & (half - 1)
                    grp = bfly - idx
                    addr_even = (grp << 1) | idx
                    addr_odd = addr_even + half

                    tw_idx = (idx << (LOG2N_16 - 1 - stg)) % (n_fft // 2)
                    tw_cos, tw_sin = fft_twiddle_lookup(tw_idx, n_fft, cos_rom_16)

                    a_re = mem_re[addr_even]
                    a_im = mem_im[addr_even]
                    b_re = mem_re[addr_odd]
                    b_im = mem_im[addr_odd]

                    prod_re = b_re * tw_cos + b_im * tw_sin
                    prod_im = b_im * tw_cos - b_re * tw_sin

                    prod_re_shifted = prod_re >> 15
                    prod_im_shifted = prod_im >> 15

                    mem_re[addr_even] = a_re + prod_re_shifted
                    mem_im[addr_even] = a_im + prod_im_shifted
                    mem_re[addr_odd] = a_re - prod_re_shifted
                    mem_im[addr_odd] = a_im - prod_im_shifted

                half <<= 1

            for n in range(n_fft):
                doppler_map_i[rbin, bin_offset + n] = saturate(mem_re[n], 16)
                doppler_map_q[rbin, bin_offset + n] = saturate(mem_im[n], 16)


    return doppler_map_i, doppler_map_q


# ===========================================================================
# Stage 3c: MTI Canceller (2-pulse, bit-accurate)
# ===========================================================================
def run_mti_canceller(decim_i, decim_q, enable=True):
    """
    Bit-accurate model of mti_canceller.v — 2-pulse canceller.

    Input:  decim_i/q — shape (N_chirps, NUM_RANGE_BINS), 16-bit signed
    Output: mti_i/q   — shape (N_chirps, NUM_RANGE_BINS), 16-bit signed

    When enable=True:
      - First chirp (chirp 0): output is all zeros (muted, no previous data)
      - Subsequent chirps: out[c][r] = current[c][r] - previous[c-1][r],
        with saturation to 16-bit.
    When enable=False:
      - Pass-through (output = input).

    RTL detail (from mti_canceller.v):
      diff_full = {I_in[15], I_in} - {prev[15], prev}  (17-bit signed)
      saturate to 16-bit: clamp to [-32768, +32767]
    """
    n_chirps, n_bins = decim_i.shape
    mti_i = np.zeros_like(decim_i)
    mti_q = np.zeros_like(decim_q)


    if not enable:
        mti_i[:] = decim_i
        mti_q[:] = decim_q
        return mti_i, mti_q

    for c in range(n_chirps):
        if c == 0:
            # First chirp: output muted (zeros) — no previous data
            mti_i[c, :] = 0
            mti_q[c, :] = 0
        else:
            for r in range(n_bins):
                # Sign-extend to 17-bit, subtract, saturate back to 16-bit
                diff_i = int(decim_i[c, r]) - int(decim_i[c - 1, r])
                diff_q = int(decim_q[c, r]) - int(decim_q[c - 1, r])
                mti_i[c, r] = saturate(diff_i, 16)
                mti_q[c, r] = saturate(diff_q, 16)

    return mti_i, mti_q


# ===========================================================================
# Stage 3d: DC Notch Filter (post-Doppler, bit-accurate)
# ===========================================================================
def run_dc_notch(doppler_i, doppler_q, width=2):
    """
    Bit-accurate model of the inline DC notch filter in radar_system_top.v.

    Input:  doppler_i/q — shape (NUM_RANGE_BINS, NUM_DOPPLER_BINS), 16-bit signed
    Output: notched_i/q — shape (NUM_RANGE_BINS, NUM_DOPPLER_BINS), 16-bit signed

    Zeros Doppler bins within ±width of DC for BOTH sub-frames.
    doppler_bin[4:0] = {sub_frame, bin[3:0]}:
      Sub-frame 0: bins 0-15,  DC = bin 0,  wrap = bin 15
      Sub-frame 1: bins 16-31, DC = bin 16, wrap = bin 31
      width=0: pass-through
      width=1: zero bins {0, 16}
      width=2: zero bins {0, 1, 15, 16, 17, 31}  etc.

    RTL logic (from radar_system_top.v):
      bin_within_sf = dop_bin[3:0]
      dc_notch_active = (width != 0) &&
                        (bin_within_sf < width || bin_within_sf > (15 - width + 1))
    """
    _n_range, n_doppler = doppler_i.shape
    notched_i = doppler_i.copy()
    notched_q = doppler_q.copy()


    if width == 0:
        return notched_i, notched_q

    zeroed_count = 0
    for dbin in range(n_doppler):
        bin_within_sf = dbin & 0xF
        active = (bin_within_sf < width) or (bin_within_sf > (15 - width + 1))
        if active:
            notched_i[:, dbin] = 0
            notched_q[:, dbin] = 0
            zeroed_count += 1

    return notched_i, notched_q


# ===========================================================================
# Stage 3e: CA-CFAR Detector (bit-accurate)
# ===========================================================================
def run_cfar_ca(doppler_i, doppler_q, guard=2, train=8,
                alpha_q44=0x30, mode='CA', _simple_threshold=500):
    """
    Bit-accurate model of cfar_ca.v — Cell-Averaging CFAR detector.

    Input:  doppler_i/q — shape (NUM_RANGE_BINS, NUM_DOPPLER_BINS), 16-bit signed
    Output: detect_flags — shape (NUM_RANGE_BINS, NUM_DOPPLER_BINS), bool
            magnitudes   — shape (NUM_RANGE_BINS, NUM_DOPPLER_BINS), uint17
            thresholds   — shape (NUM_RANGE_BINS, NUM_DOPPLER_BINS), uint17

    CFAR algorithm per Doppler column:
      1. Compute magnitude |I| + |Q| for all range bins in that column
      2. For each CUT (Cell Under Test) at range index k:
         a. Leading training cells: indices [k-G-T .. k-G-1] (clamped to valid)
         b. Lagging training cells: indices [k+G+1 .. k+G+T] (clamped to valid)
         c. noise_sum = sum of training cells (CA mode: both sides)
         d. threshold = (alpha * noise_sum) >> 4  (Q4.4 shift)
         e. detect if magnitude[k] > threshold

    RTL details (from cfar_ca.v):
      - Magnitude: |I| + |Q| (L1 norm, 17-bit unsigned)
      - Alpha in Q4.4 fixed-point (8-bit unsigned)
      - ALPHA_FRAC_BITS = 4
      - Threshold saturates to 17 bits
      - Edge handling: uses only available training cells at boundaries
      - Pipeline: ST_CFAR_THR → ST_CFAR_MUL → ST_CFAR_CMP

    Modes:
      CA: noise_sum = leading_sum + lagging_sum
      GO: noise_sum = side with greater average (cross-multiply comparison)
      SO: noise_sum = side with smaller average
    """
    n_range, n_doppler = doppler_i.shape
    ALPHA_FRAC_BITS = 4

    # Ensure train >= 1 (RTL clamps 0 → 1)
    if train == 0:
        train = 1


    # Compute magnitudes: |I| + |Q| (17-bit unsigned, matching RTL L1 norm)
    # RTL: abs_i = I[15] ? (~I + 1) : I; abs_q = Q[15] ? (~Q + 1) : Q
    # For I = -32768: ~(-32768 as 16-bit) + 1 = 32768 (unsigned)
    magnitudes = np.zeros((n_range, n_doppler), dtype=np.int64)
    for rbin in range(n_range):
        for dbin in range(n_doppler):
            i_val = int(doppler_i[rbin, dbin])
            q_val = int(doppler_q[rbin, dbin])
            abs_i = (-i_val) & 0xFFFF if i_val < 0 else i_val & 0xFFFF
            abs_q = (-q_val) & 0xFFFF if q_val < 0 else q_val & 0xFFFF
            magnitudes[rbin, dbin] = abs_i + abs_q  # 17-bit unsigned

    detect_flags = np.zeros((n_range, n_doppler), dtype=np.bool_)
    thresholds = np.zeros((n_range, n_doppler), dtype=np.int64)

    total_detections = 0

    # Process each Doppler column independently (matching RTL column-by-column)
    for dbin in range(n_doppler):
        col = magnitudes[:, dbin]  # 64 magnitudes for this Doppler bin

        for cut_idx in range(n_range):
            # Compute leading sum (cells before CUT, outside guard zone)
            leading_sum = 0
            leading_count = 0
            for t in range(1, train + 1):
                idx = cut_idx - guard - t
                if 0 <= idx < n_range:
                    leading_sum += int(col[idx])
                    leading_count += 1

            # Compute lagging sum (cells after CUT, outside guard zone)
            lagging_sum = 0
            lagging_count = 0
            for t in range(1, train + 1):
                idx = cut_idx + guard + t
                if 0 <= idx < n_range:
                    lagging_sum += int(col[idx])
                    lagging_count += 1

            # Mode-dependent noise estimate
            if mode == 'CA' or mode == 'CA-CFAR':
                noise_sum = leading_sum + lagging_sum
            elif mode == 'GO' or mode == 'GO-CFAR':
                if leading_count > 0 and lagging_count > 0:
                    if leading_sum * lagging_count > lagging_sum * leading_count:
                        noise_sum = leading_sum
                    else:
                        noise_sum = lagging_sum
                elif leading_count > 0:
                    noise_sum = leading_sum
                else:
                    noise_sum = lagging_sum
            elif mode == 'SO' or mode == 'SO-CFAR':
                if leading_count > 0 and lagging_count > 0:
                    if leading_sum * lagging_count < lagging_sum * leading_count:
                        noise_sum = leading_sum
                    else:
                        noise_sum = lagging_sum
                elif leading_count > 0:
                    noise_sum = leading_sum
                else:
                    noise_sum = lagging_sum
            else:
                noise_sum = leading_sum + lagging_sum  # Default to CA

            noise_product = alpha_q44 * noise_sum
            threshold_raw = noise_product >> ALPHA_FRAC_BITS

            # Saturate to MAG_WIDTH=17 bits
            MAX_MAG = (1 << 17) - 1  # 131071
            threshold_val = MAX_MAG if threshold_raw > MAX_MAG else int(threshold_raw)

            if int(col[cut_idx]) > threshold_val:
                detect_flags[cut_idx, dbin] = True
                total_detections += 1

            thresholds[cut_idx, dbin] = threshold_val


    return detect_flags, magnitudes, thresholds


# ===========================================================================
# Stage 4: Detection (magnitude threshold)
# ===========================================================================
def run_detection(doppler_i, doppler_q, threshold=10000):
    """
    Replicate RTL threshold detection from radar_system_top.v.
    cfar_mag = |I| + |Q| (17-bit)
    detection if cfar_mag > threshold
    """
    
    mag = np.abs(doppler_i) + np.abs(doppler_q)  # L1 norm (|I| + |Q|)
    detections = np.argwhere(mag > threshold)
    
    for d in detections[:20]:  # Print first 20
        rbin, dbin = d
        mag[rbin, dbin]
    
    if len(detections) > 20:
        pass
    
    return mag, detections


# ===========================================================================
# Stage 5: Float reference for comparison
# ===========================================================================
def run_float_reference(iq_i, iq_q):
    """
    Run the same processing in floating point for comparison.
    Uses the exact same RTL Hamming window coefficients (Q15) to isolate
    only the FFT fixed-point quantization error.
    """
    
    n_chirps, n_samples = iq_i.shape[0], iq_i.shape[1] if iq_i.ndim == 2 else len(iq_i)
    
    if iq_i.ndim == 1:
        # Single chirp — just do range FFT
        fft_out = np.fft.fft(iq_i.astype(np.float64) + 1j * iq_q.astype(np.float64))
        return np.real(fft_out), np.imag(fft_out)
    
    # Multi-chirp: range FFT per chirp, then Doppler FFT
    range_fft = np.zeros((n_chirps, n_samples), dtype=np.complex128)
    for c in range(n_chirps):
        range_fft[c, :] = np.fft.fft(iq_i[c, :] + 1j * iq_q[c, :])
    
    # Doppler FFT with RTL-identical Hamming window (Q15 coefficients as float)
    n_range = min(DOPPLER_RANGE_BINS, n_samples)
    hamming_float = np.array(HAMMING_Q15, dtype=np.float64) / 32768.0
    
    doppler_map = np.zeros((n_range, DOPPLER_TOTAL_BINS), dtype=np.complex128)
    for rbin in range(n_range):
        chirp_stack = range_fft[:DOPPLER_CHIRPS, rbin]
        for sf in range(2):
            sf_start = sf * CHIRPS_PER_SUBFRAME
            sf_end = sf_start + CHIRPS_PER_SUBFRAME
            bin_offset = sf * DOPPLER_FFT_SIZE
            windowed = chirp_stack[sf_start:sf_end] * hamming_float
            doppler_map[rbin, bin_offset:bin_offset + DOPPLER_FFT_SIZE] = np.fft.fft(windowed)
    
    return range_fft, doppler_map


# ===========================================================================
# Write hex stimulus files for RTL testbenches
# ===========================================================================
def write_hex_files(output_dir, iq_i, iq_q, prefix="stim"):
    """Write I/Q data as hex files for $readmemh in Verilog testbenches."""
    os.makedirs(output_dir, exist_ok=True)
    
    if iq_i.ndim == 1:
        n_samples = len(iq_i)
        fn_i = os.path.join(output_dir, f"{prefix}_i.hex")
        fn_q = os.path.join(output_dir, f"{prefix}_q.hex")
        
        with open(fn_i, 'w') as fi, open(fn_q, 'w') as fq:
            for n in range(n_samples):
                fi.write(signed_to_hex(int(iq_i[n]), 16) + '\n')
                fq.write(signed_to_hex(int(iq_q[n]), 16) + '\n')
        
    
    elif iq_i.ndim == 2:
        n_rows, n_cols = iq_i.shape
        # Write as flat file (row-major)
        fn_i = os.path.join(output_dir, f"{prefix}_i.hex")
        fn_q = os.path.join(output_dir, f"{prefix}_q.hex")
        
        with open(fn_i, 'w') as fi, open(fn_q, 'w') as fq:
            for r in range(n_rows):
                for c in range(n_cols):
                    fi.write(signed_to_hex(int(iq_i[r, c]), 16) + '\n')
                    fq.write(signed_to_hex(int(iq_q[r, c]), 16) + '\n')
        


def write_adc_hex(output_dir, adc_data, prefix="adc_stim"):
    """Write 8-bit unsigned ADC data as hex file."""
    os.makedirs(output_dir, exist_ok=True)
    fn = os.path.join(output_dir, f"{prefix}.hex")
    
    with open(fn, 'w') as f:
        for n in range(len(adc_data)):
            f.write(format(int(adc_data[n]) & 0xFF, '02X') + '\n')
    


# ===========================================================================
# Comparison metrics
# ===========================================================================
def compare_outputs(_name, fixed_i, fixed_q, float_i, float_q):
    """Compare fixed-point outputs against floating-point reference.
    
    Reports two metrics:
    1. Overall SNR (including saturated bins)
    2. Non-saturated SNR (excluding bins where |value| == 32767/32768)
    """
    # Ensure same length
    n = min(len(fixed_i), len(float_i))
    fi = fixed_i[:n].astype(np.float64)
    fq = fixed_q[:n].astype(np.float64)
    ri = float_i[:n].astype(np.float64)
    rq = float_q[:n].astype(np.float64)
    
    # Count saturated bins
    sat_mask = (np.abs(fi) >= 32767) | (np.abs(fq) >= 32767)
    np.sum(sat_mask)
    
    # Complex error — overall
    fixed_complex = fi + 1j * fq
    ref_complex = ri + 1j * rq
    error = fixed_complex - ref_complex
    
    signal_power = np.mean(np.abs(ref_complex) ** 2) + 1e-30
    noise_power = np.mean(np.abs(error) ** 2) + 1e-30
    10 * np.log10(signal_power / noise_power)
    np.max(np.abs(error))
    
    # Non-saturated comparison
    non_sat = ~sat_mask
    if np.any(non_sat):
        error_ns = fixed_complex[non_sat] - ref_complex[non_sat]
        sig_ns = np.mean(np.abs(ref_complex[non_sat]) ** 2) + 1e-30
        noise_ns = np.mean(np.abs(error_ns) ** 2) + 1e-30
        snr_ns = 10 * np.log10(sig_ns / noise_ns)
        np.max(np.abs(error_ns))
    else:
        snr_ns = 0.0
    
    
    return snr_ns  # Return the meaningful metric


# ===========================================================================
# Main
# ===========================================================================
def main():
    parser = argparse.ArgumentParser(description="AERIS-10 FPGA golden reference model")
    parser.add_argument('--frame', type=int, default=0, help='Frame index to process')
    parser.add_argument('--plot', action='store_true', help='Show plots')
    parser.add_argument(
        '--threshold',
        type=int,
        default=10000,
        help='Detection threshold (L1 magnitude)'
    )
    args = parser.parse_args()
    
    # Paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    fpga_dir = os.path.abspath(os.path.join(script_dir, '..', '..', '..'))
    data_base = os.path.expanduser("~/Downloads/adi_radar_data")
    amp_data = os.path.join(data_base, "amp_radar", "phaser_amp_4MSPS_500M_300u_256_m3dB.npy")
    amp_config = os.path.join(
        data_base,
        "amp_radar",
        "phaser_amp_4MSPS_500M_300u_256_m3dB_config.npy"
    )
    twiddle_1024 = os.path.join(fpga_dir, "fft_twiddle_1024.mem")
    output_dir = os.path.join(script_dir, "hex")
    
    
    # -----------------------------------------------------------------------
    # Load and quantize ADI data
    # -----------------------------------------------------------------------
    iq_i, iq_q, adc_8bit, _config = load_and_quantize_adi_data(
        amp_data, amp_config, frame_idx=args.frame
    )
    
    # iq_i, iq_q: (32, 1024) int64, 16-bit range — post-DDC equivalent
    
    # -----------------------------------------------------------------------
    # Write stimulus files
    # -----------------------------------------------------------------------
    
    # Post-DDC IQ for each chirp (for FFT + Doppler validation)
    write_hex_files(output_dir, iq_i, iq_q, "post_ddc")
    
    # Single chirp for range FFT validation
    write_hex_files(output_dir, iq_i[0], iq_q[0], "chirp0")
    
    # ADC stimulus for DDC validation
    write_adc_hex(output_dir, adc_8bit, "adc_chirp0")
    
    # -----------------------------------------------------------------------
    # Run range FFT on first chirp (bit-accurate)
    # -----------------------------------------------------------------------
    range_fft_i, range_fft_q = run_range_fft(iq_i[0], iq_q[0], twiddle_1024)
    write_hex_files(output_dir, range_fft_i, range_fft_q, "range_fft_chirp0")
    
    # Run range FFT on all 32 chirps
    all_range_i = np.zeros((DOPPLER_CHIRPS, FFT_SIZE), dtype=np.int64)
    all_range_q = np.zeros((DOPPLER_CHIRPS, FFT_SIZE), dtype=np.int64)
    
    for c in range(DOPPLER_CHIRPS):
        ri, rq = run_range_fft(iq_i[c], iq_q[c], twiddle_1024)
        all_range_i[c] = ri
        all_range_q[c] = rq
        if (c + 1) % 8 == 0:
            pass
    
    # -----------------------------------------------------------------------
    # Run Doppler FFT (bit-accurate) — "direct" path (first 64 bins)
    # -----------------------------------------------------------------------
    twiddle_16 = os.path.join(fpga_dir, "fft_twiddle_16.mem")
    doppler_i, doppler_q = run_doppler_fft(all_range_i, all_range_q, twiddle_file_16=twiddle_16)
    write_hex_files(output_dir, doppler_i, doppler_q, "doppler_map")
    
    # -----------------------------------------------------------------------
    # Run Range Bin Decimator + Doppler FFT — "full-chain" path
    # This models the actual RTL data flow:
    #   range FFT → range_bin_decimator (peak detection) → Doppler
    # -----------------------------------------------------------------------
    
    decim_i, decim_q = run_range_bin_decimator(
        all_range_i, all_range_q,
        mode=1, start_bin=0,
        input_bins=FFT_SIZE, output_bins=DOPPLER_RANGE_BINS,
        decimation_factor=FFT_SIZE // DOPPLER_RANGE_BINS
    )
    
    # Write full-chain range FFT input: all 32 chirps x 1024 bins = 32768 samples
    # This is the stimulus for the range_bin_decimator in the full-chain testbench.
    # Format: packed {Q[31:16], I[15:0]} per RTL range_data bus format
    fc_input_file = os.path.join(output_dir, "fullchain_range_input.hex")
    with open(fc_input_file, 'w') as f:
        for c in range(DOPPLER_CHIRPS):
            for b in range(FFT_SIZE):
                i_val = int(all_range_i[c, b]) & 0xFFFF
                q_val = int(all_range_q[c, b]) & 0xFFFF
                packed = (q_val << 16) | i_val
                f.write(f"{packed:08X}\n")
    
    # Write decimated output reference for standalone decimator test
    write_hex_files(output_dir, decim_i, decim_q, "decimated_range")
    
    # Now run Doppler on the decimated data — this is the full-chain reference
    fc_doppler_i, fc_doppler_q = run_doppler_fft(
        decim_i, decim_q, twiddle_file_16=twiddle_16
    )
    write_hex_files(output_dir, fc_doppler_i, fc_doppler_q, "fullchain_doppler_ref")
    
    # Write full-chain Doppler reference as packed 32-bit for easy RTL comparison
    fc_doppler_packed_file = os.path.join(output_dir, "fullchain_doppler_ref_packed.hex")
    with open(fc_doppler_packed_file, 'w') as f:
        for rbin in range(DOPPLER_RANGE_BINS):
            for dbin in range(DOPPLER_TOTAL_BINS):
                i_val = int(fc_doppler_i[rbin, dbin]) & 0xFFFF
                q_val = int(fc_doppler_q[rbin, dbin]) & 0xFFFF
                packed = (q_val << 16) | i_val
                f.write(f"{packed:08X}\n")
    
    # Save numpy arrays for the full-chain path
    np.save(os.path.join(output_dir, "decimated_range_i.npy"), decim_i)
    np.save(os.path.join(output_dir, "decimated_range_q.npy"), decim_q)
    np.save(os.path.join(output_dir, "fullchain_doppler_i.npy"), fc_doppler_i)
    np.save(os.path.join(output_dir, "fullchain_doppler_q.npy"), fc_doppler_q)
    
    # -----------------------------------------------------------------------
    # Full-chain with MTI + DC Notch + CFAR
    # This models the complete RTL data flow:
    #   range FFT → decimator → MTI canceller → Doppler → DC notch → CFAR
    # -----------------------------------------------------------------------
    mti_i, mti_q = run_mti_canceller(decim_i, decim_q, enable=True)
    write_hex_files(output_dir, mti_i, mti_q, "fullchain_mti_ref")
    np.save(os.path.join(output_dir, "fullchain_mti_i.npy"), mti_i)
    np.save(os.path.join(output_dir, "fullchain_mti_q.npy"), mti_q)
    
    # Doppler on MTI-filtered data
    mti_doppler_i, mti_doppler_q = run_doppler_fft(
        mti_i, mti_q, twiddle_file_16=twiddle_16
    )
    write_hex_files(output_dir, mti_doppler_i, mti_doppler_q, "fullchain_mti_doppler_ref")
    np.save(os.path.join(output_dir, "fullchain_mti_doppler_i.npy"), mti_doppler_i)
    np.save(os.path.join(output_dir, "fullchain_mti_doppler_q.npy"), mti_doppler_q)
    
    # DC notch on MTI-Doppler data
    DC_NOTCH_WIDTH = 2  # Default test value: zero bins {0, 1, 31}
    notched_i, notched_q = run_dc_notch(mti_doppler_i, mti_doppler_q, width=DC_NOTCH_WIDTH)
    write_hex_files(output_dir, notched_i, notched_q, "fullchain_notched_ref")
    
    # Write notched Doppler as packed 32-bit for RTL comparison
    fc_notched_packed_file = os.path.join(output_dir, "fullchain_notched_ref_packed.hex")
    with open(fc_notched_packed_file, 'w') as f:
        for rbin in range(DOPPLER_RANGE_BINS):
            for dbin in range(DOPPLER_TOTAL_BINS):
                i_val = int(notched_i[rbin, dbin]) & 0xFFFF
                q_val = int(notched_q[rbin, dbin]) & 0xFFFF
                packed = (q_val << 16) | i_val
                f.write(f"{packed:08X}\n")
    
    # CFAR on DC-notched data
    CFAR_GUARD = 2
    CFAR_TRAIN = 8
    CFAR_ALPHA = 0x30  # Q4.4 = 3.0
    CFAR_MODE = 'CA'
    cfar_flags, cfar_mag, cfar_thr = run_cfar_ca(
        notched_i, notched_q,
        guard=CFAR_GUARD, train=CFAR_TRAIN,
        alpha_q44=CFAR_ALPHA, mode=CFAR_MODE
    )
    
    # Write CFAR reference files
    # 1. Magnitude map (17-bit unsigned, row-major: 64 range x 32 Doppler = 2048)
    cfar_mag_file = os.path.join(output_dir, "fullchain_cfar_mag.hex")
    with open(cfar_mag_file, 'w') as f:
        for rbin in range(DOPPLER_RANGE_BINS):
            for dbin in range(DOPPLER_TOTAL_BINS):
                m = int(cfar_mag[rbin, dbin]) & 0x1FFFF
                f.write(f"{m:05X}\n")
    
    # 2. Threshold map (17-bit unsigned)
    cfar_thr_file = os.path.join(output_dir, "fullchain_cfar_thr.hex")
    with open(cfar_thr_file, 'w') as f:
        for rbin in range(DOPPLER_RANGE_BINS):
            for dbin in range(DOPPLER_TOTAL_BINS):
                t = int(cfar_thr[rbin, dbin]) & 0x1FFFF
                f.write(f"{t:05X}\n")
    
    # 3. Detection flags (1-bit per cell)
    cfar_det_file = os.path.join(output_dir, "fullchain_cfar_det.hex")
    with open(cfar_det_file, 'w') as f:
        for rbin in range(DOPPLER_RANGE_BINS):
            for dbin in range(DOPPLER_TOTAL_BINS):
                d = 1 if cfar_flags[rbin, dbin] else 0
                f.write(f"{d:01X}\n")
    
    # 4. Detection list (text)
    cfar_detections = np.argwhere(cfar_flags)
    cfar_det_list_file = os.path.join(output_dir, "fullchain_cfar_detections.txt")
    with open(cfar_det_list_file, 'w') as f:
        f.write("# AERIS-10 Full-Chain CFAR Detection List\n")
        f.write(f"# Chain: decim -> MTI -> Doppler -> DC notch(w={DC_NOTCH_WIDTH}) -> CA-CFAR\n")
        f.write(
            f"# CFAR: guard={CFAR_GUARD}, train={CFAR_TRAIN}, "
            f"alpha=0x{CFAR_ALPHA:02X}, mode={CFAR_MODE}\n"
        )
        f.write("# Format: range_bin doppler_bin magnitude threshold\n")
        for det in cfar_detections:
            r, d = det
            f.write(f"{r} {d} {cfar_mag[r, d]} {cfar_thr[r, d]}\n")
    
    # Save numpy arrays
    np.save(os.path.join(output_dir, "fullchain_cfar_mag.npy"), cfar_mag)
    np.save(os.path.join(output_dir, "fullchain_cfar_thr.npy"), cfar_thr)
    np.save(os.path.join(output_dir, "fullchain_cfar_flags.npy"), cfar_flags)
    
    # Run detection on full-chain Doppler map
    fc_mag, fc_detections = run_detection(fc_doppler_i, fc_doppler_q, threshold=args.threshold)
    
    # Save full-chain detection reference
    fc_det_file = os.path.join(output_dir, "fullchain_detections.txt")
    with open(fc_det_file, 'w') as f:
        f.write("# AERIS-10 Full-Chain Golden Reference Detections\n")
        f.write(f"# Threshold: {args.threshold}\n")
        f.write("# Format: range_bin doppler_bin magnitude\n")
        for d in fc_detections:
            rbin, dbin = d
            f.write(f"{rbin} {dbin} {fc_mag[rbin, dbin]}\n")
    
    # Also write detection reference as hex for RTL comparison
    fc_det_mag_file = os.path.join(output_dir, "fullchain_detection_mag.hex")
    with open(fc_det_mag_file, 'w') as f:
        for rbin in range(DOPPLER_RANGE_BINS):
            for dbin in range(DOPPLER_TOTAL_BINS):
                m = int(fc_mag[rbin, dbin]) & 0x1FFFF  # 17-bit unsigned
                f.write(f"{m:05X}\n")
    
    # -----------------------------------------------------------------------
    # Run detection on direct-path Doppler map (for backward compatibility)
    # -----------------------------------------------------------------------
    mag, detections = run_detection(doppler_i, doppler_q, threshold=args.threshold)
    
    # Save detection list
    det_file = os.path.join(output_dir, "detections.txt")
    with open(det_file, 'w') as f:
        f.write("# AERIS-10 Golden Reference Detections\n")
        f.write(f"# Threshold: {args.threshold}\n")
        f.write("# Format: range_bin doppler_bin magnitude\n")
        for d in detections:
            rbin, dbin = d
            f.write(f"{rbin} {dbin} {mag[rbin, dbin]}\n")
    
    # -----------------------------------------------------------------------
    # Float reference and comparison
    # -----------------------------------------------------------------------
    
    range_fft_float, doppler_float = run_float_reference(iq_i, iq_q)
    
    # Compare range FFT (chirp 0)
    float_range_i = np.real(range_fft_float[0, :]).astype(np.float64)
    float_range_q = np.imag(range_fft_float[0, :]).astype(np.float64)
    compare_outputs("Range FFT", range_fft_i, range_fft_q,
                                float_range_i, float_range_q)
    
    # Compare Doppler map
    float_doppler_i = np.real(doppler_float).flatten().astype(np.float64)
    float_doppler_q = np.imag(doppler_float).flatten().astype(np.float64)
    compare_outputs("Doppler FFT", 
                                   doppler_i.flatten(), doppler_q.flatten(),
                                   float_doppler_i, float_doppler_q)
    
    # -----------------------------------------------------------------------
    # Save numpy reference outputs
    # -----------------------------------------------------------------------
    np.save(os.path.join(output_dir, "range_fft_all_i.npy"), all_range_i)
    np.save(os.path.join(output_dir, "range_fft_all_q.npy"), all_range_q)
    np.save(os.path.join(output_dir, "doppler_map_i.npy"), doppler_i)
    np.save(os.path.join(output_dir, "doppler_map_q.npy"), doppler_q)
    np.save(os.path.join(output_dir, "detection_mag.npy"), mag)
    
    # -----------------------------------------------------------------------
    # Summary
    # -----------------------------------------------------------------------
    
    # -----------------------------------------------------------------------
    # Optional plots
    # -----------------------------------------------------------------------
    if args.plot:
        try:
            import matplotlib.pyplot as plt
            
            _fig, axes = plt.subplots(2, 2, figsize=(14, 10))
            
            # Range FFT magnitude (chirp 0)
            range_mag = np.sqrt(range_fft_i.astype(float)**2 + range_fft_q.astype(float)**2)
            axes[0, 0].plot(20 * np.log10(range_mag + 1))
            axes[0, 0].set_title("Range FFT Magnitude (Chirp 0)")
            axes[0, 0].set_xlabel("Range Bin")
            axes[0, 0].set_ylabel("dB")
            axes[0, 0].grid(True)
            
            # Range-Doppler map
            rd_mag = np.sqrt(doppler_i.astype(float)**2 + doppler_q.astype(float)**2)
            rd_db = 20 * np.log10(rd_mag + 1)
            im = axes[0, 1].imshow(rd_db, aspect='auto', origin='lower',
                                    cmap='viridis')
            axes[0, 1].set_title("Range-Doppler Map (Fixed-Point)")
            axes[0, 1].set_xlabel("Doppler Bin")
            axes[0, 1].set_ylabel("Range Bin")
            plt.colorbar(im, ax=axes[0, 1], label="dB")
            
            # Float reference Range-Doppler map
            float_rd_mag = np.abs(doppler_float)
            float_rd_db = 20 * np.log10(float_rd_mag + 1)
            im2 = axes[1, 0].imshow(float_rd_db, aspect='auto', origin='lower',
                                     cmap='viridis')
            axes[1, 0].set_title("Range-Doppler Map (Float Reference)")
            axes[1, 0].set_xlabel("Doppler Bin")
            axes[1, 0].set_ylabel("Range Bin")
            plt.colorbar(im2, ax=axes[1, 0], label="dB")
            
            # Detection overlay
            axes[1, 1].imshow(rd_db, aspect='auto', origin='lower', cmap='viridis')
            if len(detections) > 0:
                axes[1, 1].scatter(detections[:, 1], detections[:, 0],
                                   c='red', marker='x', s=50, linewidths=2)
            axes[1, 1].set_title(f"Detections (threshold={args.threshold})")
            axes[1, 1].set_xlabel("Doppler Bin")
            axes[1, 1].set_ylabel("Range Bin")
            
            plt.tight_layout()
            plot_file = os.path.join(output_dir, "golden_reference_plots.png")
            plt.savefig(plot_file, dpi=150)
            plt.show()
            
        except ImportError:
            pass


if __name__ == "__main__":
    main()
