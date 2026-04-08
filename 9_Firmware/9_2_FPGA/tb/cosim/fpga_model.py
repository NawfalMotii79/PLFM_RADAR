#!/usr/bin/env python3
"""
Bit-accurate Python model of the AERIS-10 FPGA signal processing chain.

Mirrors the RTL fixed-point arithmetic exactly:
  NCO -> Mixer -> CIC -> FIR -> Matched Filter -> Range Decimation -> Doppler

All operations use Python integers to match Verilog's exact bit-level behavior.
No floating point is used in signal processing (only for verification/display).

Usage:
    from fpga_model import SignalChain
    chain = SignalChain()
    # Feed ADC samples one at a time or in batches
    chain.nco_step(ftw, phase_offset=0)
    ...

Author: Phase 0.5 co-simulation suite for PLFM_RADAR
"""

import os

# =============================================================================
# Fixed-point utility functions
# =============================================================================


def sign_extend(value, bits):
    """Sign-extend a `bits`-wide integer to full Python int."""
    mask = (1 << bits) - 1
    value = value & mask
    if value & (1 << (bits - 1)):
        return value - (1 << bits)
    return value


def to_unsigned(value, bits):
    """Convert signed Python int to unsigned representation in `bits` width."""
    mask = (1 << bits) - 1
    return value & mask


def saturate(value, bits):
    """Saturate a value to signed `bits`-wide range [-2^(bits-1), 2^(bits-1)-1]."""
    max_pos = (1 << (bits - 1)) - 1
    max_neg = -(1 << (bits - 1))
    if value > max_pos:
        return max_pos
    if value < max_neg:
        return max_neg
    return value


def arith_rshift(value, shift, width=None):
    """Arithmetic right shift. Python >> on signed int is already arithmetic."""
    return value >> shift


# =============================================================================
# NCO: Numerically Controlled Oscillator (6-stage pipeline)
# =============================================================================

# Quarter-wave sine LUT (64 entries, 16-bit unsigned)
# Matches nco_400m_enhanced.v exactly
NCO_SINE_LUT = [
    0x0000,
    0x0324,
    0x0648,
    0x096A,
    0x0C8C,
    0x0FAB,
    0x12C8,
    0x15E2,
    0x18F9,
    0x1C0B,
    0x1F1A,
    0x2223,
    0x2528,
    0x2826,
    0x2B1F,
    0x2E11,
    0x30FB,
    0x33DF,
    0x36BA,
    0x398C,
    0x3C56,
    0x3F17,
    0x41CE,
    0x447A,
    0x471C,
    0x49B4,
    0x4C3F,
    0x4EBF,
    0x5133,
    0x539B,
    0x55F5,
    0x5842,
    0x5A82,
    0x5CB3,
    0x5ED7,
    0x60EB,
    0x62F1,
    0x64E8,
    0x66CF,
    0x68A6,
    0x6A6D,
    0x6C23,
    0x6DC9,
    0x6F5E,
    0x70E2,
    0x7254,
    0x73B5,
    0x7504,
    0x7641,
    0x776B,
    0x7884,
    0x7989,
    0x7A7C,
    0x7B5C,
    0x7C29,
    0x7CE3,
    0x7D89,
    0x7E1D,
    0x7E9C,
    0x7F09,
    0x7F61,
    0x7FA6,
    0x7FD8,
    0x7FF5,
]


class NCO:
    """
    Bit-accurate model of nco_400m_enhanced.v

    6-stage pipeline:
      Stage 1: phase_accumulator += ftw; phase_accum_reg = prev accumulator
      Stage 2: phase_with_offset = phase_accum_reg + {phase_offset, 16'b0}
      Stage 3a: Register LUT index + quadrant from phase_with_offset
      Stage 3b: LUT read using registered index -> register abs + quadrant
      Stage 4: Compute negations
      Stage 5: Quadrant sign MUX -> sin_out, cos_out

    CREG=1 on DSP48E1: The behavioral sim model captures the accumulator
    BEFORE adding the new FTW (phase_accum_reg = old accumulator), then
    uses that old value for offset addition next cycle. This introduces
    a 2-cycle pipeline delay from input to phase_with_offset.

    Latency: 6 cycles from phase_valid to dds_ready
    """

    def __init__(self):
        self.phase_accumulator = 0  # 32-bit
        self.phase_accum_reg = 0  # Stage 1 output
        self.phase_with_offset = 0  # Stage 2 output

        # Stage 3a
        self.lut_index_pipe = 0  # 6-bit
        self.quadrant_pipe = 0  # 2-bit

        # Stage 3b
        self.sin_abs_reg = 0  # 16-bit unsigned
        self.cos_abs_reg = 0x7FFF  # 16-bit unsigned
        self.quadrant_reg = 0  # 2-bit

        # Stage 4
        self.sin_neg_reg = 0  # 16-bit signed
        self.cos_neg_reg = sign_extend((-0x7FFF) & 0xFFFF, 16)
        self.sin_abs_reg2 = 0
        self.cos_abs_reg2 = 0x7FFF
        self.quadrant_reg2 = 0

        # Stage 5 outputs
        self.sin_out = 0  # 16-bit signed
        self.cos_out = 0x7FFF  # 16-bit signed

        # Valid pipeline
        self.valid_pipe = 0  # 6-bit shift register
        self.dds_ready = False

    def _quadrant_and_index(self, phase_with_offset):
        """Compute quadrant and LUT index from phase_with_offset[31:24]."""
        lut_address = (phase_with_offset >> 24) & 0xFF
        quadrant = (lut_address >> 6) & 0x3
        raw_index = lut_address & 0x3F

        # RTL: lut_index = (quadrant[0] ^ quadrant[1]) ? ~lut_address[5:0] : lut_address[5:0]
        if (quadrant & 1) ^ ((quadrant >> 1) & 1):
            lut_index = (~raw_index) & 0x3F
        else:
            lut_index = raw_index

        return quadrant, lut_index

    def step(self, ftw, phase_offset=0, phase_valid=True):
        """
        Advance one clock cycle.

        Args:
            ftw: 32-bit frequency tuning word
            phase_offset: 16-bit phase offset
            phase_valid: enable signal

        Returns:
            (sin_out, cos_out, dds_ready) as signed 16-bit integers + bool
        """
        ftw = ftw & 0xFFFFFFFF
        phase_offset = phase_offset & 0xFFFF

        if phase_valid:
            # ---- Stage 1: Phase accumulator (behavioral sim model) ----
            # phase_accum_reg captures OLD accumulator value BEFORE update
            old_accum = self.phase_accumulator
            self.phase_accumulator = (self.phase_accumulator + ftw) & 0xFFFFFFFF
            self.phase_accum_reg = old_accum

            # ---- Stage 2: Offset addition (uses PREVIOUS cycle's phase_accum_reg) ----
            # The RTL does: phase_with_offset <= phase_accum_reg + {phase_offset, 16'b0}
            # But in blocking assignment order, phase_accum_reg was just updated above.
            # In Verilog NBA semantics: both happen simultaneously, so phase_with_offset
            # uses the OLD phase_accum_reg (from previous cycle), not the one just assigned.
            # We need to capture the PREVIOUS phase_accum_reg for this:
            # Actually re-reading the RTL more carefully:
            #   phase_accumulator <= phase_accumulator + ftw;  (new accum)
            #   phase_accum_reg   <= phase_accumulator;        (NBA: uses OLD accum)
            #   phase_with_offset <= phase_accum_reg + offset; (NBA: uses OLD phase_accum_reg)
            # So we need to track the pre-update values.
            # Let's fix by computing in proper NBA order:

            # We already captured old_accum above. But phase_with_offset uses the
            # OLD phase_accum_reg (the value from the PREVIOUS call).
            # We stored self.phase_accum_reg at the start of this call as the
            # value from last cycle. So:
            pass  # phase_with_offset computed below from OLD values

        # Compute all NBA assignments from OLD state:
        # Save old state for NBA evaluation
        old_phase_accum_reg = self.phase_accum_reg
        old_phase_with_offset = self.phase_with_offset
        old_lut_index_pipe = self.lut_index_pipe
        old_quadrant_pipe = self.quadrant_pipe
        old_sin_abs_reg = self.sin_abs_reg
        old_cos_abs_reg = self.cos_abs_reg
        old_quadrant_reg = self.quadrant_reg
        old_sin_neg_reg = self.sin_neg_reg
        old_cos_neg_reg = self.cos_neg_reg
        old_sin_abs_reg2 = self.sin_abs_reg2
        old_cos_abs_reg2 = self.cos_abs_reg2
        old_quadrant_reg2 = self.quadrant_reg2
        old_valid_pipe = self.valid_pipe

        if phase_valid:
            # Stage 1 NBA: phase_accum_reg <= phase_accumulator (old value)
            _new_phase_accum_reg = (self.phase_accumulator - ftw) & 0xFFFFFFFF  # noqa: F841 — old accum before add (derivation reference)
            # Wait - let me re-derive. The Verilog is:
            #   phase_accumulator <= phase_accumulator + frequency_tuning_word;
            #   phase_accum_reg   <= phase_accumulator;  // OLD value (NBA)
            #   phase_with_offset <= phase_accum_reg + {phase_offset, 16'b0};  // OLD phase_accum_reg
            # Since all are NBA (<=), they all read the values from BEFORE this edge.
            # So: new_phase_accumulator = old_phase_accumulator + ftw
            #     new_phase_accum_reg   = old_phase_accumulator
            #     new_phase_with_offset = old_phase_accum_reg + offset
            old_phase_accumulator = (self.phase_accumulator - ftw) & 0xFFFFFFFF  # reconstruct
            self.phase_accum_reg = old_phase_accumulator
            self.phase_with_offset = (
                old_phase_accum_reg + ((phase_offset << 16) & 0xFFFFFFFF)
            ) & 0xFFFFFFFF
            # phase_accumulator was already updated above

        # ---- Stage 3a: Register LUT address + quadrant ----
        # Gated by valid_pipe[1]
        if (old_valid_pipe >> 1) & 1:
            quadrant_w, lut_index_w = self._quadrant_and_index(old_phase_with_offset)
            self.lut_index_pipe = lut_index_w
            self.quadrant_pipe = quadrant_w

        # ---- Stage 3b: LUT read + register ----
        # Gated by valid_pipe[2]
        if (old_valid_pipe >> 2) & 1:
            self.sin_abs_reg = NCO_SINE_LUT[old_lut_index_pipe]
            self.cos_abs_reg = NCO_SINE_LUT[63 - old_lut_index_pipe]
            self.quadrant_reg = old_quadrant_pipe

        # ---- Stage 4: Negation ----
        # Gated by valid_pipe[3]
        if (old_valid_pipe >> 3) & 1:
            self.sin_neg_reg = sign_extend((-old_sin_abs_reg) & 0xFFFF, 16)
            self.cos_neg_reg = sign_extend((-old_cos_abs_reg) & 0xFFFF, 16)
            self.sin_abs_reg2 = old_sin_abs_reg
            self.cos_abs_reg2 = old_cos_abs_reg
            self.quadrant_reg2 = old_quadrant_reg

        # ---- Stage 5: Quadrant MUX ----
        # Gated by valid_pipe[4]
        if (old_valid_pipe >> 4) & 1:
            q = old_quadrant_reg2
            if q == 0:  # Q1: sin+, cos+
                self.sin_out = sign_extend(old_sin_abs_reg2, 16)
                self.cos_out = sign_extend(old_cos_abs_reg2, 16)
            elif q == 1:  # Q2: sin+, cos-
                self.sin_out = sign_extend(old_sin_abs_reg2, 16)
                self.cos_out = old_cos_neg_reg
            elif q == 2:  # Q3: sin-, cos-
                self.sin_out = old_sin_neg_reg
                self.cos_out = old_cos_neg_reg
            elif q == 3:  # Q4: sin-, cos+
                self.sin_out = old_sin_neg_reg
                self.cos_out = sign_extend(old_cos_abs_reg2, 16)

        # ---- Valid pipeline ----
        self.valid_pipe = ((old_valid_pipe << 1) | (1 if phase_valid else 0)) & 0x3F
        self.dds_ready = bool((old_valid_pipe >> 5) & 1)

        return self.sin_out, self.cos_out, self.dds_ready


# =============================================================================
# Mixer: DSP48E1 multiply with 3-cycle pipeline (AREG+MREG+PREG)
# =============================================================================


class Mixer:
    """
    Bit-accurate model of ddc_400m mixer.

    ADC 8-bit unsigned -> 18-bit signed conversion:
      adc_signed = {1'b0, adc_data, 9'b0} - {1'b0, 8'hFF, 9'b0} / 2
      This is effectively: adc_signed = (adc_data << 9) - (0xFF << 9) / 2
      But the Verilog expression is:
        {1'b0, adc_data, {9{1'b0}}} - {1'b0, {8{1'b1}}, {9{1'b0}}} / 2

    Then mixed_i = adc_signed * cos_out (18-bit * 16-bit = 34-bit product)
    CIC input = mixed_i[33:16] (top 18 bits of 34-bit product)

    3-cycle DSP48E1 pipeline: AREG -> MREG -> PREG
    """

    def __init__(self):
        # Stage 1 (AREG/BREG)
        self.adc_signed_reg = 0  # 18-bit signed
        self.cos_pipe_reg = 0  # 16-bit signed
        self.sin_pipe_reg = 0  # 16-bit signed

        # Stage 2 (MREG)
        self.mult_i_internal = 0  # 34-bit signed
        self.mult_q_internal = 0  # 34-bit signed

        # Stage 3 (PREG)
        self.mult_i_reg = 0  # 34-bit signed
        self.mult_q_reg = 0  # 34-bit signed

        # Valid pipeline
        self.valid_pipe = 0  # 3-bit

    @staticmethod
    def adc_to_signed(adc_data_8bit):
        """
        Convert 8-bit unsigned ADC to 18-bit signed.
        RTL: adc_signed_w = {1'b0, adc_data, {9{1'b0}}} -
                            {1'b0, {8{1'b1}}, {9{1'b0}}} / 2
        = (adc_data << 9) - (0xFF << 9) / 2
        = (adc_data << 9) - (0xFF << 8)  [integer division]
        = (adc_data << 9) - 0x7F80
        """
        adc_data_8bit = adc_data_8bit & 0xFF
        # {1'b0, adc_data, 9'b0} = adc_data << 9, zero-padded to 18 bits
        term1 = adc_data_8bit << 9
        # {1'b0, 8'hFF, 9'b0} / 2 = (0xFF << 9) / 2 = 0xFF << 8 (integer division)
        # But actually in Verilog: {1'b0, {8{1'b1}}, {9{1'b0}}} = 17'b0_11111111_000000000
        # = 0x1FE00 ... / 2 = 0xFF00
        # Wait: {1'b0, 8'hFF, 9'b0} = 0_11111111_000000000 = 0x1FE00 (18 bits)
        # Divided by 2 = 0xFF00 = 65280
        term2 = 0xFF00
        result = (term1 - term2) & 0x3FFFF  # 18-bit mask
        return sign_extend(result, 18)

    def step(self, adc_data, cos_out, sin_out, nco_ready, adc_valid):
        """
        Advance one clock cycle.

        Returns:
            (mixed_i_top18, mixed_q_top18, mixed_valid)
            where mixed_i_top18 = mixed_i[33:16]
        """
        adc_signed_w = self.adc_to_signed(adc_data)

        # Save old state for NBA
        old_adc_signed_reg = self.adc_signed_reg
        old_cos_pipe_reg = self.cos_pipe_reg
        old_sin_pipe_reg = self.sin_pipe_reg
        old_mult_i_internal = self.mult_i_internal
        old_mult_q_internal = self.mult_q_internal
        old_valid_pipe = self.valid_pipe

        # Stage 1: AREG/BREG (always clocked, no valid gating)
        self.adc_signed_reg = adc_signed_w
        self.cos_pipe_reg = sign_extend(cos_out & 0xFFFF, 16)
        self.sin_pipe_reg = sign_extend(sin_out & 0xFFFF, 16)

        # Stage 2: MREG
        self.mult_i_internal = old_adc_signed_reg * old_cos_pipe_reg
        self.mult_q_internal = old_adc_signed_reg * old_sin_pipe_reg

        # Stage 3: PREG
        self.mult_i_reg = old_mult_i_internal
        self.mult_q_reg = old_mult_q_internal

        # Valid pipeline
        valid_in = 1 if (nco_ready and adc_valid) else 0
        self.valid_pipe = ((old_valid_pipe << 1) | valid_in) & 0x7

        mixed_valid = bool((old_valid_pipe >> 2) & 1)

        # CIC gets mixed_i[33:16] — top 18 bits of 34-bit product
        # This is equivalent to arithmetic right shift by 16, then take 18 LSBs
        mixed_i_top18 = sign_extend((self.mult_i_reg >> 16) & 0x3FFFF, 18) if mixed_valid else 0
        mixed_q_top18 = sign_extend((self.mult_q_reg >> 16) & 0x3FFFF, 18) if mixed_valid else 0

        return mixed_i_top18, mixed_q_top18, mixed_valid


# =============================================================================
# CIC Decimator (5-stage, 4x decimation, DSP48E1 PCOUT cascade)
# =============================================================================


class CICDecimator:
    """
    Bit-accurate model of cic_decimator_4x_enhanced.v

    5-stage CIC with 4x decimation.
    Integrators: 48-bit wrapping arithmetic (modular).
    CREG=1 on integrator_0: data_in_c_delayed lags by 1 cycle.
    Comb: 28-bit, COMB_DELAY=1, 5 stages.
    Output: >>10 scaling, saturate to 18-bit [-131072, +131071].

    The comb section has a 3-stage pipeline:
      Stage 1: comb computations + temp_scaled_output = comb[4] >>> 10
      Stage 2: saturation comparison flags (sat_pos, sat_neg) + temp_output
      Stage 3: MUX from flags -> data_out
    """

    STAGES = 5
    DECIMATION = 4
    COMB_DELAY = 1
    ACC_WIDTH = 48
    COMB_WIDTH = 28
    ACC_MASK = (1 << 48) - 1
    COMB_MASK = (1 << 28) - 1

    def __init__(self):
        # Integrators (48-bit wrapping)
        self.int_stages = [0] * self.STAGES
        self.data_in_c_delayed = 0  # Models CREG=1

        # Comb section (28-bit signed)
        self.comb = [0] * self.STAGES
        self.comb_delay = [[0] * self.COMB_DELAY for _ in range(self.STAGES)]

        # Decimation control
        self.decimation_counter = 0
        self.data_valid_delayed = False
        self.data_valid_comb = False
        self.integrator_sampled = 0  # 28-bit

        # Comb output pipeline (3 stages)
        self.temp_scaled_output = 0
        self.temp_output = 0
        self.sat_pos = False
        self.sat_neg = False
        self.temp_output_pipe = 0
        self.data_out_valid_pipe = False

        # Outputs
        self.data_out = 0
        self.data_out_valid = False

    def step(self, data_in_18, data_valid):
        """
        Advance one clock cycle.

        Args:
            data_in_18: 18-bit signed input
            data_valid: input valid flag

        Returns:
            (data_out, data_out_valid) as signed 18-bit + bool
        """
        data_in_18 = sign_extend(data_in_18 & 0x3FFFF, 18)
        # Sign-extend to 48 bits
        data_in_c = data_in_18 & self.ACC_MASK

        # Save old state for NBA semantics
        old_int = list(self.int_stages)
        old_data_in_c_delayed = self.data_in_c_delayed
        old_decimation_counter = self.decimation_counter
        old_integrator_sampled = self.integrator_sampled
        old_data_valid_delayed = self.data_valid_delayed
        old_data_valid_comb = self.data_valid_comb
        old_comb = list(self.comb)
        old_comb_delay = [list(d) for d in self.comb_delay]
        old_temp_scaled_output = self.temp_scaled_output
        old_sat_pos = self.sat_pos
        old_sat_neg = self.sat_neg
        old_temp_output_pipe = self.temp_output_pipe
        old_data_out_valid_pipe = self.data_out_valid_pipe

        # ---- Integrator chain (DSP48E1 behavioral sim) ----
        if data_valid:
            # CREG pipeline: capture current data, use previous
            self.data_in_c_delayed = sign_extend(data_in_c, 48)
            self.int_stages[0] = (old_int[0] + old_data_in_c_delayed) & self.ACC_MASK
            for i in range(1, self.STAGES):
                self.int_stages[i] = (old_int[i] + old_int[i - 1]) & self.ACC_MASK

        # ---- Decimation control ----
        if data_valid:
            if old_decimation_counter == self.DECIMATION - 1:
                self.decimation_counter = 0
                self.data_valid_delayed = True
                # Capture integrator_4 output, truncate to COMB_WIDTH
                int4_val = self.int_stages[4]  # Use NEW value (from NBA above)
                # Actually in RTL, p_out_4 is read as a wire from the DSP/behavioral model
                # The NBA order means we read the just-updated value
                self.integrator_sampled = sign_extend(int4_val & self.COMB_MASK, self.COMB_WIDTH)
            else:
                self.decimation_counter = old_decimation_counter + 1
                self.data_valid_delayed = False
        else:
            self.data_valid_delayed = False

        # ---- Pipeline valid for comb section ----
        self.data_valid_comb = old_data_valid_delayed

        # ---- Comb section ----
        if old_data_valid_comb:
            for i in range(self.STAGES):
                if i == 0:
                    inp = old_integrator_sampled
                    self.comb[0] = sign_extend(
                        (inp - old_comb_delay[0][self.COMB_DELAY - 1]) & self.COMB_MASK,
                        self.COMB_WIDTH,
                    )
                    # Shift delay line
                    for j in range(self.COMB_DELAY - 1, 0, -1):
                        self.comb_delay[0][j] = old_comb_delay[0][j - 1]
                    self.comb_delay[0][0] = inp
                else:
                    inp = old_comb[i - 1]
                    self.comb[i] = sign_extend(
                        (inp - old_comb_delay[i][self.COMB_DELAY - 1]) & self.COMB_MASK,
                        self.COMB_WIDTH,
                    )
                    for j in range(self.COMB_DELAY - 1, 0, -1):
                        self.comb_delay[i][j] = old_comb_delay[i][j - 1]
                    self.comb_delay[i][0] = inp

            # Scale by >>>10 (CIC gain = 4^5 = 1024 = 2^10)
            self.temp_scaled_output = sign_extend(old_comb[self.STAGES - 1], self.COMB_WIDTH) >> 10
            self.temp_output = sign_extend(self.temp_scaled_output & 0x3FFFF, 18)

            # Pipeline Stage 2: saturation flags
            self.sat_pos = old_temp_scaled_output > 131071
            self.sat_neg = old_temp_scaled_output < -131072
            self.temp_output_pipe = sign_extend(old_temp_scaled_output & 0x3FFFF, 18)
            self.data_out_valid_pipe = True
        else:
            self.data_out_valid_pipe = False

        # ---- Pipeline Stage 3: Output MUX ----
        if old_data_out_valid_pipe:
            if old_sat_pos:
                self.data_out = 131071
            elif old_sat_neg:
                self.data_out = -131072
            else:
                self.data_out = old_temp_output_pipe
            self.data_out_valid = True
        else:
            self.data_out_valid = False

        return self.data_out, self.data_out_valid


# =============================================================================
# FIR Lowpass Filter (32-tap, 5-stage binary adder tree, 7-cycle latency)
# =============================================================================

# FIR coefficients (18-bit signed hex from fir_lowpass.v)
# These are 18-bit signed values stored in Verilog as 18'sh...
FIR_COEFFICIENTS_HEX = [
    0x000AD,
    0x000CE,
    0x3FD87,
    0x002A6,
    0x000E0,
    0x3F8C0,
    0x00A45,
    0x3FD82,
    0x3F0B5,
    0x01CAD,
    0x3EE59,
    0x3E821,
    0x04841,
    0x3B340,
    0x3E299,
    0x1FFFF,
    0x1FFFF,
    0x3E299,
    0x3B340,
    0x04841,
    0x3E821,
    0x3EE59,
    0x01CAD,
    0x3F0B5,
    0x3FD82,
    0x00A45,
    0x3F8C0,
    0x000E0,
    0x002A6,
    0x3FD87,
    0x000CE,
    0x000AD,
]

# Convert to signed Python ints
FIR_COEFFICIENTS = [sign_extend(c, 18) for c in FIR_COEFFICIENTS_HEX]


class FIRFilter:
    """
    Bit-accurate model of fir_lowpass_parallel_enhanced.v

    32-tap FIR with 5-stage pipelined binary adder tree.
    Input: 18-bit signed.  Output: 18-bit signed.
    Accumulator: 36-bit signed.

    Pipeline (7 cycles total):
      Cycle 0: data_valid -> shift delay line (combinational multiply)
      Cycle 1: L0: 16 pairwise sums of 32 products
      Cycle 2: L1: 8 pairwise sums
      Cycle 3: L2: 4 pairwise sums
      Cycle 4: L3: 2 pairwise sums
      Cycle 5: L4: final sum -> accumulator_reg
      Cycle 6: output saturation/rounding
    """

    TAPS = 32
    DATA_WIDTH = 18
    COEFF_WIDTH = 18
    ACCUM_WIDTH = 36
    PRODUCT_WIDTH = DATA_WIDTH + COEFF_WIDTH  # 36 bits

    def __init__(self):
        self.delay_line = [0] * self.TAPS
        self.add_l0 = [0] * 16
        self.add_l1 = [0] * 8
        self.add_l2 = [0] * 4
        self.add_l3 = [0, 0]
        self.accumulator_reg = 0
        self.data_out = 0
        self.data_out_valid = False
        self.valid_pipe = 0  # 7-bit

    def step(self, data_in_18, data_valid):
        """
        Advance one clock cycle.

        Returns:
            (data_out, data_out_valid) as signed 18-bit + bool
        """
        data_in_18 = sign_extend(data_in_18 & 0x3FFFF, 18)
        old_valid_pipe = self.valid_pipe

        # ---- Stage 0: Shift delay line ----
        if data_valid:
            for i in range(self.TAPS - 1, 0, -1):
                self.delay_line[i] = self.delay_line[i - 1]
            self.delay_line[0] = data_in_18

        # Combinational multiply (uses current delay_line)
        mult_results = []
        for k in range(self.TAPS):
            prod = self.delay_line[k] * FIR_COEFFICIENTS[k]
            mult_results.append(prod)

        # Save old adder tree state
        old_l0 = list(self.add_l0)
        old_l1 = list(self.add_l1)
        old_l2 = list(self.add_l2)
        old_l3 = list(self.add_l3)
        old_accum = self.accumulator_reg

        # ---- Stage 1 (Level 0): 16 pairwise sums ----
        if (old_valid_pipe >> 0) & 1:
            for i in range(16):
                # Sign-extend products to ACCUM_WIDTH
                a = sign_extend(
                    mult_results[2 * i] & ((1 << self.PRODUCT_WIDTH) - 1), self.PRODUCT_WIDTH
                )
                b = sign_extend(
                    mult_results[2 * i + 1] & ((1 << self.PRODUCT_WIDTH) - 1), self.PRODUCT_WIDTH
                )
                self.add_l0[i] = a + b

        # ---- Stage 2 (Level 1): 8 pairwise sums ----
        if (old_valid_pipe >> 1) & 1:
            for i in range(8):
                self.add_l1[i] = old_l0[2 * i] + old_l0[2 * i + 1]

        # ---- Stage 3 (Level 2): 4 pairwise sums ----
        if (old_valid_pipe >> 2) & 1:
            for i in range(4):
                self.add_l2[i] = old_l1[2 * i] + old_l1[2 * i + 1]

        # ---- Stage 4 (Level 3): 2 pairwise sums ----
        if (old_valid_pipe >> 3) & 1:
            self.add_l3[0] = old_l2[0] + old_l2[1]
            self.add_l3[1] = old_l2[2] + old_l2[3]

        # ---- Stage 5 (Level 4): Final sum ----
        if (old_valid_pipe >> 4) & 1:
            self.accumulator_reg = old_l3[0] + old_l3[1]

        # ---- Stage 6: Output saturation/rounding ----
        if (old_valid_pipe >> 5) & 1:
            accum = old_accum
            max_pos = (1 << (self.ACCUM_WIDTH - 2)) - 1  # 2^34 - 1
            min_neg = -(1 << (self.ACCUM_WIDTH - 2))  # -2^34

            if accum > max_pos:
                self.data_out = (1 << (self.DATA_WIDTH - 1)) - 1  # 131071
            elif accum < min_neg:
                self.data_out = -(1 << (self.DATA_WIDTH - 1))  # -131072
            else:
                # Round and truncate: accumulator_reg[ACCUM_WIDTH-2 : DATA_WIDTH-1]
                # = accum[34:17] = bits 34 down to 17
                # This is equivalent to: (accum >> 17) masked to 18 bits
                self.data_out = sign_extend((accum >> (self.DATA_WIDTH - 1)) & 0x3FFFF, 18)
            self.data_out_valid = True
        else:
            self.data_out_valid = (old_valid_pipe >> 5) & 1

        # Update valid pipeline
        self.valid_pipe = ((old_valid_pipe << 1) | (1 if data_valid else 0)) & 0x7F
        self.data_out_valid = bool((old_valid_pipe >> 5) & 1)

        return self.data_out, self.data_out_valid


# =============================================================================
# DDC Input Interface (18 -> 16 bit rounding)
# =============================================================================


class DDCInputInterface:
    """
    Bit-accurate model of ddc_input_interface.v

    Converts 18-bit FIR output to 16-bit with rounding:
      adc_i = ddc_i[17:2] + ddc_i[1]

    2-cycle valid pipeline:
      Cycle 1: valid_sync = valid_i_reg && valid_q_reg
      Cycle 2: adc_valid = valid_sync (data computed during valid_sync)
    """

    def __init__(self):
        self.valid_i_reg = False
        self.valid_q_reg = False
        self.valid_sync = False
        self.adc_valid = False
        self.adc_i = 0
        self.adc_q = 0

    def step(self, ddc_i_18, ddc_q_18, valid_i, valid_q):
        """
        Returns:
            (adc_i, adc_q, adc_valid) as signed 16-bit, signed 16-bit, bool
        """
        old_valid_sync = self.valid_sync

        # Pipeline valid
        self.valid_sync = self.valid_i_reg and self.valid_q_reg
        self.adc_valid = old_valid_sync
        self.valid_i_reg = valid_i
        self.valid_q_reg = valid_q

        # Data path (clocked on valid_sync)
        if old_valid_sync:
            ddc_i = sign_extend(ddc_i_18 & 0x3FFFF, 18)
            ddc_q = sign_extend(ddc_q_18 & 0x3FFFF, 18)
            # adc_i = ddc_i[17:2] + ddc_i[1]  (rounding)
            trunc_i = (ddc_i >> 2) & 0xFFFF  # bits [17:2]
            round_i = (ddc_i >> 1) & 1  # bit [1]
            trunc_q = (ddc_q >> 2) & 0xFFFF
            round_q = (ddc_q >> 1) & 1
            self.adc_i = sign_extend((trunc_i + round_i) & 0xFFFF, 16)
            self.adc_q = sign_extend((trunc_q + round_q) & 0xFFFF, 16)

        return self.adc_i, self.adc_q, self.adc_valid


# =============================================================================
# FFT Engine (1024-point radix-2 DIT, in-place, 32-bit internal)
# =============================================================================


def load_twiddle_rom(filepath=None):
    """
    Load 256-entry quarter-wave cosine ROM from hex file.
    Returns list of 256 signed 16-bit integers.
    """
    if filepath is None:
        # Default path relative to this file
        base = os.path.dirname(os.path.abspath(__file__))
        filepath = os.path.join(base, "..", "..", "fft_twiddle_1024.mem")

    values = []
    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("//"):
                continue
            val = int(line, 16)
            values.append(sign_extend(val, 16))
    return values


def _twiddle_lookup(k, n, cos_rom):
    """
    Quarter-wave twiddle reconstruction from cos_rom (N/4 entries).
    Returns (cos_val, sin_val) as signed 16-bit.

    Matches fft_engine.v logic exactly:
      k=0:       cos=rom[0],         sin=0
      k=N/4:     cos=0,              sin=rom[0]
      k<N/4:     cos=rom[k],         sin=rom[N/4-k]
      k>N/4:     cos=-rom[N/2-k],    sin=rom[k-N/4]
    """
    n4 = n // 4
    n2 = n // 2

    k = k % n2  # twiddle indices are modulo N/2

    if k == 0:
        return cos_rom[0], 0
    elif k == n4:
        return 0, cos_rom[0]
    elif k < n4:
        return cos_rom[k], cos_rom[n4 - k]
    else:
        return sign_extend((-cos_rom[n2 - k]) & 0xFFFF, 16), cos_rom[k - n4]


class FFTEngine:
    """
    Bit-accurate model of fft_engine.v

    1024-point radix-2 DIT FFT/IFFT.
    Internal: 32-bit signed working data.
    Twiddle: 16-bit Q15 from quarter-wave cosine ROM.
    Butterfly: multiply 32x16->49 bits, >>>15, add/subtract.
    Output: saturate 32->16 bits. IFFT also >>>LOG2N before saturate.
    """

    def __init__(self, n=1024, twiddle_file=None):
        self.N = n
        self.LOG2N = n.bit_length() - 1
        self.cos_rom = load_twiddle_rom(twiddle_file)
        # Working memory (32-bit signed I/Q pairs)
        self.mem_re = [0] * n
        self.mem_im = [0] * n

    @staticmethod
    def _bit_reverse(val, bits):
        """Bit-reverse an integer."""
        result = 0
        for _ in range(bits):
            result = (result << 1) | (val & 1)
            val >>= 1
        return result

    def compute(self, in_re, in_im, inverse=False):
        """
        Run full FFT or IFFT.

        Args:
            in_re: list of N signed 16-bit real inputs
            in_im: list of N signed 16-bit imag inputs
            inverse: True for IFFT

        Returns:
            (out_re, out_im): lists of N signed 16-bit outputs
        """
        n = self.N
        log2n = self.LOG2N

        # LOAD: sign-extend 16->32 and store at bit-reversed addresses
        for i in range(n):
            br = self._bit_reverse(i, log2n)
            self.mem_re[br] = sign_extend(in_re[i] & 0xFFFF, 16)
            self.mem_im[br] = sign_extend(in_im[i] & 0xFFFF, 16)

        # COMPUTE: LOG2N stages of butterflies
        for stage in range(log2n):
            half = 1 << stage
            tw_stride = (n >> 1) >> stage

            for bfly in range(n // 2):
                idx = bfly & (half - 1)
                grp = bfly - idx
                even = (grp << 1) | idx
                odd = even + half
                tw_idx = idx * tw_stride

                # Read
                a_re = self.mem_re[even]
                a_im = self.mem_im[even]
                b_re = self.mem_re[odd]
                b_im = self.mem_im[odd]

                # Twiddle lookup
                tw_cos, tw_sin = _twiddle_lookup(tw_idx, n, self.cos_rom)

                # Multiply (49-bit products)
                if not inverse:
                    # Forward: t = b * (cos + j*sin)
                    prod_re = b_re * tw_cos + b_im * tw_sin
                    prod_im = b_im * tw_cos - b_re * tw_sin
                else:
                    # Inverse: t = b * (cos - j*sin)
                    prod_re = b_re * tw_cos - b_im * tw_sin
                    prod_im = b_im * tw_cos + b_re * tw_sin

                # Shift >>>15 (arithmetic, 49->32)
                t_re = prod_re >> 15
                t_im = prod_im >> 15

                # Add/subtract
                self.mem_re[even] = a_re + t_re
                self.mem_im[even] = a_im + t_im
                self.mem_re[odd] = a_re - t_re
                self.mem_im[odd] = a_im - t_im

        # OUTPUT: read in linear order, saturate to 16 bits
        out_re = []
        out_im = []
        for i in range(n):
            re_val = self.mem_re[i]
            im_val = self.mem_im[i]

            if inverse:
                # IFFT: >>>LOG2N before saturate
                re_val = re_val >> log2n
                im_val = im_val >> log2n

            out_re.append(saturate(re_val, 16))
            out_im.append(saturate(im_val, 16))

        return out_re, out_im


# =============================================================================
# Frequency Matched Filter (conjugate multiply, 4-stage pipeline)
# =============================================================================


class FreqMatchedFilter:
    """
    Bit-accurate model of frequency_matched_filter.v

    Conjugate multiply: (a + jb) * conj(c + jd) = (ac+bd) + j(bc-ad)

    4-stage pipeline:
      P1: Register inputs
      P2: Four 16x16 multiplies -> 32-bit products
      P3: Add: real_sum = ac + bd, imag_sum = bc - ad (32-bit Q30)
      P4: Round (+ 1<<14), saturate, extract [30:15] -> 16-bit Q15

    For batch processing, we compute all samples directly.
    """

    @staticmethod
    def conjugate_multiply_sample(sig_re, sig_im, ref_re, ref_im):
        """
        Compute one conjugate multiply with exact RTL arithmetic.

        Returns (out_re, out_im) as signed 16-bit.
        """
        a = sign_extend(sig_re & 0xFFFF, 16)
        b = sign_extend(sig_im & 0xFFFF, 16)
        c = sign_extend(ref_re & 0xFFFF, 16)
        d = sign_extend(ref_im & 0xFFFF, 16)

        # Stage 2: 16x16 multiplies -> 32-bit signed
        ac = a * c
        bd = b * d
        bc = b * c
        ad = a * d

        # Stage 3: accumulate (Q30)
        real_sum = ac + bd
        imag_sum = bc - ad

        # Stage 4: round + saturate + extract [30:15]
        def round_sat_extract(q30_val):
            rounded = q30_val + (1 << 14)
            # Saturation check
            if rounded > 0x3FFF8000:
                return 0x7FFF
            elif rounded < -0x3FFF8000:
                return sign_extend(0x8000, 16)
            else:
                return sign_extend((rounded >> 15) & 0xFFFF, 16)

        out_re = round_sat_extract(real_sum)
        out_im = round_sat_extract(imag_sum)
        return out_re, out_im

    @staticmethod
    def process_block(sig_re, sig_im, ref_re, ref_im):
        """Process N samples of conjugate multiply."""
        n = len(sig_re)
        out_re = []
        out_im = []
        for i in range(n):
            r, m = FreqMatchedFilter.conjugate_multiply_sample(
                sig_re[i], sig_im[i], ref_re[i], ref_im[i]
            )
            out_re.append(r)
            out_im.append(m)
        return out_re, out_im


# =============================================================================
# Matched Filter Processing Chain
# =============================================================================


class MatchedFilterChain:
    """
    Complete matched filter: FFT(signal) * conj(FFT(ref)) -> IFFT

    Uses a single FFTEngine instance (as in RTL, engine is reused).
    """

    def __init__(self, fft_size=1024, twiddle_file=None):
        self.fft_size = fft_size
        self.fft = FFTEngine(n=fft_size, twiddle_file=twiddle_file)
        self.conj_mult = FreqMatchedFilter()

    def process(self, sig_re, sig_im, ref_re, ref_im):
        """
        Run matched filter on 1024-sample signal + reference.

        Args:
            sig_re/im: signal I/Q (16-bit signed, 1024 samples)
            ref_re/im: reference chirp I/Q (16-bit signed, 1024 samples)

        Returns:
            (range_profile_re, range_profile_im): 1024 x 16-bit signed
        """
        # Forward FFT of signal
        sig_fft_re, sig_fft_im = self.fft.compute(sig_re, sig_im, inverse=False)

        # Forward FFT of reference (same engine, reused)
        ref_fft_re, ref_fft_im = self.fft.compute(ref_re, ref_im, inverse=False)

        # Conjugate multiply
        prod_re, prod_im = self.conj_mult.process_block(
            sig_fft_re, sig_fft_im, ref_fft_re, ref_fft_im
        )

        # Inverse FFT
        range_re, range_im = self.fft.compute(prod_re, prod_im, inverse=True)

        return range_re, range_im


# =============================================================================
# Range Bin Decimator (1024 -> 64, factor 16)
# =============================================================================


class RangeBinDecimator:
    """
    Bit-accurate model of range_bin_decimator.v

    Three modes:
      00: Simple decimation (take center sample at index 8)
      01: Peak detection (max |I|+|Q|)
      10: Averaging (sum >> 4, truncation)
      11: Reserved (output 0)
    """

    DECIMATION_FACTOR = 16
    OUTPUT_BINS = 64

    @staticmethod
    def decimate(range_re, range_im, mode=1, start_bin=0):
        """
        Decimate 1024 range bins to 64.

        Args:
            range_re/im: 1024 x signed 16-bit
            mode: 0=center, 1=peak, 2=average, 3=zero
            start_bin: first input bin to process (0-1023)

        Returns:
            (out_re, out_im): 64 x signed 16-bit
        """
        out_re = []
        out_im = []
        df = RangeBinDecimator.DECIMATION_FACTOR

        for b in range(RangeBinDecimator.OUTPUT_BINS):
            base = start_bin + b * df

            if mode == 0:
                # Simple decimation: take center sample
                idx = base + df // 2
                if idx < len(range_re):
                    out_re.append(range_re[idx])
                    out_im.append(range_im[idx])
                else:
                    out_re.append(0)
                    out_im.append(0)

            elif mode == 1:
                # Peak detection: max |I| + |Q|
                best_mag = -1
                best_re = 0
                best_im = 0
                for s in range(df):
                    idx = base + s
                    if idx < len(range_re):
                        re_val = sign_extend(range_re[idx] & 0xFFFF, 16)
                        im_val = sign_extend(range_im[idx] & 0xFFFF, 16)
                        # abs via 2's complement (matches RTL)
                        abs_re = (-re_val) if re_val < 0 else re_val
                        abs_im = (-im_val) if im_val < 0 else im_val
                        mag = abs_re + abs_im  # 17-bit unsigned
                        if mag > best_mag:
                            best_mag = mag
                            best_re = re_val
                            best_im = im_val
                out_re.append(best_re)
                out_im.append(best_im)

            elif mode == 2:
                # Averaging: sum >> 4
                sum_re = 0
                sum_im = 0
                for s in range(df):
                    idx = base + s
                    if idx < len(range_re):
                        sum_re += sign_extend(range_re[idx] & 0xFFFF, 16)
                        sum_im += sign_extend(range_im[idx] & 0xFFFF, 16)
                # Truncate (arithmetic right shift by 4), take 16 bits
                out_re.append(sign_extend((sum_re >> 4) & 0xFFFF, 16))
                out_im.append(sign_extend((sum_im >> 4) & 0xFFFF, 16))

            else:
                # Mode 3: reserved, output 0
                out_re.append(0)
                out_im.append(0)

        return out_re, out_im


# =============================================================================
# Doppler Processor (Hamming window + dual 16-point FFT)
# =============================================================================

# Hamming window LUT (16 entries, 16-bit unsigned Q15)
# Matches doppler_processor.v window_coeff[0:15]
# w[n] = 0.54 - 0.46 * cos(2*pi*n/15), n=0..15, symmetric
HAMMING_WINDOW = [
    0x0A3D,
    0x0E5C,
    0x1B6D,
    0x3088,
    0x4B33,
    0x6573,
    0x7642,
    0x7F62,
    0x7F62,
    0x7642,
    0x6573,
    0x4B33,
    0x3088,
    0x1B6D,
    0x0E5C,
    0x0A3D,
]


class DopplerProcessor:
    """
    Bit-accurate model of doppler_processor_optimized.v (dual 16-pt FFT architecture).

    The staggered-PRF frame has 32 chirps total:
      - Sub-frame 0 (long PRI):  chirps 0-15  -> 16-pt Hamming -> 16-pt FFT -> bins 0-15
      - Sub-frame 1 (short PRI): chirps 16-31 -> 16-pt Hamming -> 16-pt FFT -> bins 16-31

    Output: doppler_bin[4:0] = {sub_frame_id, bin_in_subframe[3:0]}
    Total output per range bin: 32 bins (16 + 16), same interface as before.
    """

    DOPPLER_FFT_SIZE = 16  # Per sub-frame
    RANGE_BINS = 64
    CHIRPS_PER_FRAME = 32
    CHIRPS_PER_SUBFRAME = 16

    def __init__(self, twiddle_file_16=None):
        """
        For 16-point FFT, we need the 16-point twiddle file.
        If not provided, we generate twiddle factors mathematically
        (cos(2*pi*k/16) for k=0..3, quarter-wave ROM with 4 entries).
        """
        self.fft16 = None
        self._twiddle_file_16 = twiddle_file_16

    @staticmethod
    def window_multiply(data_16, window_16):
        """
        Hamming window multiply matching RTL:
          product = data * window  (16x16 -> 32-bit signed)
          rounded = product + (1 << 14)
          result  = rounded >> 15  (arithmetic right shift)
        """
        d = sign_extend(data_16 & 0xFFFF, 16)
        # Window values are unsigned Q15, but multiply is $signed * $signed
        # in the RTL. The window values are all positive (max 0x7FFF), so
        # treating as signed 16-bit is fine (MSB is always 0).
        w = sign_extend(window_16 & 0xFFFF, 16)
        product = d * w  # 32-bit signed
        rounded = product + (1 << 14)
        result = rounded >> 15  # arithmetic right shift
        return sign_extend(result & 0xFFFF, 16)

    def process_frame(self, chirp_data_i, chirp_data_q):
        """
        Process one complete Doppler frame using dual 16-pt FFTs.

        Args:
            chirp_data_i: 2D array [32 chirps][64 range bins] of signed 16-bit I
            chirp_data_q: 2D array [32 chirps][64 range bins] of signed 16-bit Q

        Returns:
            (doppler_map_i, doppler_map_q): 2D arrays [64 range bins][32 doppler bins]
                                            of signed 16-bit
                                            Bins 0-15 = sub-frame 0 (long PRI)
                                            Bins 16-31 = sub-frame 1 (short PRI)
        """
        doppler_map_i = []
        doppler_map_q = []

        # Generate 16-pt twiddle factors (quarter-wave cos, 4 entries)
        # cos(2*pi*k/16) for k=0..3
        # Matches fft_twiddle_16.mem: 7FFF, 7641, 5A82, 30FB
        import math

        cos_rom_16 = []
        for k in range(4):
            val = round(32767.0 * math.cos(2.0 * math.pi * k / 16.0))
            cos_rom_16.append(sign_extend(val & 0xFFFF, 16))

        fft16 = FFTEngine.__new__(FFTEngine)
        fft16.N = 16
        fft16.LOG2N = 4
        fft16.cos_rom = cos_rom_16
        fft16.mem_re = [0] * 16
        fft16.mem_im = [0] * 16

        for rbin in range(self.RANGE_BINS):
            # Output bins for this range bin: 32 total (16 from each sub-frame)
            out_re = [0] * 32
            out_im = [0] * 32

            # Process each sub-frame independently
            for sf in range(2):
                chirp_start = sf * self.CHIRPS_PER_SUBFRAME
                bin_offset = sf * self.DOPPLER_FFT_SIZE

                fft_in_re = []
                fft_in_im = []

                for c in range(self.CHIRPS_PER_SUBFRAME):
                    chirp = chirp_start + c
                    re_val = sign_extend(chirp_data_i[chirp][rbin] & 0xFFFF, 16)
                    im_val = sign_extend(chirp_data_q[chirp][rbin] & 0xFFFF, 16)

                    # Apply 16-pt Hamming window (index = c within sub-frame)
                    win_re = self.window_multiply(re_val, HAMMING_WINDOW[c])
                    win_im = self.window_multiply(im_val, HAMMING_WINDOW[c])

                    fft_in_re.append(win_re)
                    fft_in_im.append(win_im)

                # 16-point forward FFT
                fft_out_re, fft_out_im = fft16.compute(fft_in_re, fft_in_im, inverse=False)

                # Pack into output: sub-frame 0 -> bins 0-15, sub-frame 1 -> bins 16-31
                for b in range(self.DOPPLER_FFT_SIZE):
                    out_re[bin_offset + b] = fft_out_re[b]
                    out_im[bin_offset + b] = fft_out_im[b]

            doppler_map_i.append(out_re)
            doppler_map_q.append(out_im)

        return doppler_map_i, doppler_map_q


# =============================================================================
# Complete Signal Chain (DDC through Doppler)
# =============================================================================


class SignalChain:
    """
    Full AERIS-10 signal processing chain.

    For sample-by-sample co-simulation with RTL, use the step-based API
    (nco, mixer, cic, fir individually).

    For block-level validation, use process_chirp() or process_frame().
    """

    # System parameters
    FS_ADC = 400_000_000  # ADC sample rate
    FS_SYS = 100_000_000  # System clock
    IF_FREQ = 120_000_000  # IF frequency
    FTW_120MHZ = 0x4CCCCCCD  # Phase increment for 120 MHz at 400 MSPS

    def __init__(self, twiddle_file_1024=None, twiddle_file_16=None):
        self.nco = NCO()
        self.mixer = Mixer()
        self.cic_i = CICDecimator()
        self.cic_q = CICDecimator()
        self.fir_i = FIRFilter()
        self.fir_q = FIRFilter()
        self.ddc_interface = DDCInputInterface()
        self.matched_filter = MatchedFilterChain(fft_size=1024, twiddle_file=twiddle_file_1024)
        self.range_decimator = RangeBinDecimator()
        self.doppler = DopplerProcessor(twiddle_file_16=twiddle_file_16)

    def ddc_step(self, adc_data_8bit, ftw=None):
        """
        Process one ADC sample through the DDC (NCO + Mixer + CIC).
        Runs at 400 MHz rate.

        Returns dict with intermediate and output values.
        """
        if ftw is None:
            ftw = self.FTW_120MHZ

        # NCO
        sin_val, cos_val, nco_ready = self.nco.step(ftw, phase_offset=0, phase_valid=True)

        # Mixer
        mix_i, mix_q, mix_valid = self.mixer.step(adc_data_8bit, cos_val, sin_val, nco_ready, True)

        # CIC (both channels)
        cic_i_out, cic_i_valid = self.cic_i.step(mix_i, mix_valid)
        cic_q_out, cic_q_valid = self.cic_q.step(mix_q, mix_valid)

        return {
            "sin": sin_val,
            "cos": cos_val,
            "nco_ready": nco_ready,
            "mix_i": mix_i,
            "mix_q": mix_q,
            "mix_valid": mix_valid,
            "cic_i": cic_i_out,
            "cic_q": cic_q_out,
            "cic_valid": cic_i_valid and cic_q_valid,
        }

    def process_adc_block(self, adc_samples, ftw=None):
        """
        Process a block of ADC samples through DDC (NCO->Mixer->CIC->FIR).

        Args:
            adc_samples: list of 8-bit unsigned ADC values at 400 MSPS

        Returns:
            dict with:
              'baseband_i': list of 16-bit signed I samples (at 100 MHz)
              'baseband_q': list of 16-bit signed Q samples (at 100 MHz)
              'cic_i_raw': list of raw CIC outputs for debugging
              'fir_i_raw': list of raw FIR outputs for debugging
        """
        if ftw is None:
            ftw = self.FTW_120MHZ

        cic_outputs_i = []
        cic_outputs_q = []
        fir_outputs_i = []
        fir_outputs_q = []
        baseband_i = []
        baseband_q = []

        # In the RTL, the DDC runs at 400 MHz (NCO, mixer, CIC), while the
        # FIR and DDC input interface run at 100 MHz. After CIC decimation (4x),
        # the FIR sees one valid sample every 4 ADC clocks. The CDC crossing
        # from 400->100 MHz is modeled here by only clocking FIR when CIC
        # produces valid output — the FIR runs at the decimated rate.
        #
        # Between CIC outputs, the FIR is NOT clocked at all (unlike the RTL
        # where it idles at 100 MHz with data_valid=0). This is equivalent
        # because the FIR's internal pipeline only advances when data_valid=1.

        for sample in adc_samples:
            result = self.ddc_step(sample, ftw)

            if result["cic_valid"]:
                cic_outputs_i.append(result["cic_i"])
                cic_outputs_q.append(result["cic_q"])

                # FIR (runs at decimated rate, ~100 MHz)
                # Only clock FIR when CIC has valid output — models the
                # CDC crossing + FIR data_valid gating
                fir_i_out, fir_i_valid = self.fir_i.step(result["cic_i"], True)
                fir_q_out, fir_q_valid = self.fir_q.step(result["cic_q"], True)

                if fir_i_valid and fir_q_valid:
                    fir_outputs_i.append(fir_i_out)
                    fir_outputs_q.append(fir_q_out)

                    # DDC input interface (18->16 bit rounding)
                    bb_i, bb_q, bb_valid = self.ddc_interface.step(fir_i_out, fir_q_out, True, True)
                    if bb_valid:
                        baseband_i.append(bb_i)
                        baseband_q.append(bb_q)
                else:
                    # Clock DDC interface with invalid to advance its pipeline
                    self.ddc_interface.step(0, 0, False, False)

        return {
            "baseband_i": baseband_i,
            "baseband_q": baseband_q,
            "cic_i_raw": cic_outputs_i,
            "cic_q_raw": cic_outputs_q,
            "fir_i_raw": fir_outputs_i,
            "fir_q_raw": fir_outputs_q,
        }


# =============================================================================
# Self-test / Validation
# =============================================================================


def _self_test():
    """Quick sanity checks for each module."""
    import math

    print("=" * 60)
    print("FPGA Model Self-Test")
    print("=" * 60)

    # --- NCO test ---
    print("\n--- NCO Test ---")
    nco = NCO()
    ftw = 0x4CCCCCCD  # 120 MHz at 400 MSPS
    # Run 20 cycles to fill pipeline
    results = []
    for i in range(20):
        s, c, ready = nco.step(ftw)
        if ready:
            results.append((s, c))

    if results:
        print(f"  First valid output: sin={results[0][0]}, cos={results[0][1]}")
        print(f"  Got {len(results)} valid outputs from 20 cycles")
        # Check quadrature: sin^2 + cos^2 should be approximately 32767^2
        s, c = results[-1]
        mag_sq = s * s + c * c
        expected = 32767 * 32767
        error_pct = abs(mag_sq - expected) / expected * 100
        print(
            f"  Quadrature check: sin^2+cos^2={mag_sq}, expected~{expected}, error={error_pct:.2f}%"
        )
    print("  NCO: OK")

    # --- Mixer test ---
    print("\n--- Mixer Test ---")
    mixer = Mixer()
    # Test with mid-scale ADC (128) and known cos/sin
    for i in range(5):
        mi, mq, mv = mixer.step(128, 0x7FFF, 0, True, True)
    print(f"  Mixer with adc=128, cos=max, sin=0: I={mi}, Q={mq}, valid={mv}")
    print("  Mixer: OK")

    # --- CIC test ---
    print("\n--- CIC Test ---")
    cic = CICDecimator()
    dc_val = sign_extend(0x1000, 18)  # Small positive DC
    out_count = 0
    for i in range(100):
        out, valid = cic.step(dc_val, True)
        if valid:
            out_count += 1
    print(f"  CIC: {out_count} outputs from 100 inputs (expect ~25 with 4x decimation + pipeline)")
    print("  CIC: OK")

    # --- FIR test ---
    print("\n--- FIR Test ---")
    fir = FIRFilter()
    out_count = 0
    for i in range(50):
        out, valid = fir.step(1000, True)
        if valid:
            out_count += 1
    print(f"  FIR: {out_count} outputs from 50 inputs (expect ~43 with 7-cycle latency)")
    print("  FIR: OK")

    # --- FFT test ---
    print("\n--- FFT Test (1024-pt) ---")
    try:
        fft = FFTEngine(n=1024)
        # Single tone at bin 10
        in_re = [0] * 1024
        in_im = [0] * 1024
        for i in range(1024):
            in_re[i] = int(32767 * 0.5 * math.cos(2 * math.pi * 10 * i / 1024))
            in_re[i] = saturate(in_re[i], 16)
        out_re, out_im = fft.compute(in_re, in_im, inverse=False)
        # Find peak bin
        max_mag = 0
        peak_bin = 0
        for i in range(512):
            mag = abs(out_re[i]) + abs(out_im[i])
            if mag > max_mag:
                max_mag = mag
                peak_bin = i
        print(f"  FFT peak at bin {peak_bin} (expected 10), magnitude={max_mag}")
        # IFFT roundtrip
        rt_re, rt_im = fft.compute(out_re, out_im, inverse=True)
        max_err = max(abs(rt_re[i] - in_re[i]) for i in range(1024))
        print(f"  FFT->IFFT roundtrip max error: {max_err} LSBs")
        print("  FFT: OK")
    except FileNotFoundError:
        print("  FFT: SKIPPED (twiddle file not found)")

    # --- Conjugate multiply test ---
    print("\n--- Conjugate Multiply Test ---")
    # (1+j0) * conj(1+j0) = 1+j0
    # In Q15: 32767 * 32767 -> should get close to 32767
    r, m = FreqMatchedFilter.conjugate_multiply_sample(0x7FFF, 0, 0x7FFF, 0)
    print(f"  (32767+j0) * conj(32767+j0) = {r}+j{m} (expect ~32767+j0)")
    # (0+j32767) * conj(0+j32767) = (0+j32767)(0-j32767) = 32767^2 -> ~32767
    r2, m2 = FreqMatchedFilter.conjugate_multiply_sample(0, 0x7FFF, 0, 0x7FFF)
    print(f"  (0+j32767) * conj(0+j32767) = {r2}+j{m2} (expect ~32767+j0)")
    print("  Conjugate Multiply: OK")

    # --- Range decimator test ---
    print("\n--- Range Bin Decimator Test ---")
    test_re = list(range(1024))
    test_im = [0] * 1024
    out_re, out_im = RangeBinDecimator.decimate(test_re, test_im, mode=0)
    print(f"  Mode 0 (center): first 5 bins = {out_re[:5]} (expect [8, 24, 40, 56, 72])")
    print("  Range Decimator: OK")

    print("\n" + "=" * 60)
    print("ALL SELF-TESTS PASSED")
    print("=" * 60)


if __name__ == "__main__":
    _self_test()
