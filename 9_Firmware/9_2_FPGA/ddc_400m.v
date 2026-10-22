`timescale 1ns / 1ps

module ddc_400m_enhanced (
    input wire clk_400m,           // 400MHz clock from ADC DCO
    input wire clk_100m,           // 100MHz system clock
    input wire reset_n,
    input wire mixers_enable,
    input wire [7:0] adc_data,     // ADC data at 400MHz
    input wire adc_data_valid_i,     // Valid at 400MHz
	 input wire adc_data_valid_q,
    output wire signed [17:0] baseband_i,
    output wire signed [17:0] baseband_q,  
    output wire baseband_valid_i,
	 output wire baseband_valid_q,

    output wire [1:0] ddc_status,
    // Enhanced interfaces
    output wire [7:0] ddc_diagnostics,
    output wire mixer_saturation,
    output wire filter_overflow,
	 
	 input wire [1:0] test_mode,
    input wire [15:0] test_phase_inc,
    input wire force_saturation,
    input wire reset_monitors,
    output wire [31:0] debug_sample_count,
    output wire [17:0] debug_internal_i,
    output wire [17:0] debug_internal_q
);

// Parameters for numerical precision
parameter ADC_WIDTH = 8;
parameter NCO_WIDTH = 16;
parameter MIXER_WIDTH = 18;
parameter OUTPUT_WIDTH = 18;

// IF frequency parameters
parameter IF_FREQ = 120000000;
parameter FS = 400000000;
parameter PHASE_WIDTH = 32;

// Internal signals
wire signed [15:0] sin_out, cos_out;
wire nco_ready;
wire cic_valid;
wire fir_valid;
wire [17:0] cic_i_out, cic_q_out;
wire signed [17:0] fir_i_out, fir_q_out;


// Diagnostic registers
reg [2:0] saturation_count;
reg overflow_detected;
reg [7:0] error_counter;

// Debug monitoring signals
reg [31:0] sample_counter;
wire signed [17:0] debug_mixed_i_trunc;
wire signed [17:0] debug_mixed_q_trunc;

// Real-time status monitoring
reg [7:0] signal_power_i, signal_power_q;

// Internal mixing signals
// Pipeline: NCO fabric reg (1) + DSP48E1 AREG/BREG (1) + MREG (1) + PREG (1) + retiming (1) = 5 cycles
// The NCO fabric pipeline register was added to break the long NCO→DSP B-port route
// (1.505ns routing in Build 26, WNS=+0.002ns). With BREG=1 still active inside the DSP,
// total latency increases by 1 cycle (2.5ns at 400MHz — negligible for radar).
wire signed [MIXER_WIDTH-1:0] adc_signed_w;
reg signed [MIXER_WIDTH + NCO_WIDTH -1:0] mixed_i, mixed_q;
reg mixed_valid;
reg mixer_overflow_i, mixer_overflow_q;
// Pipeline valid tracking: 5-stage shift register (1 NCO pipe + 3 DSP48E1 + 1 retiming)
reg [4:0] dsp_valid_pipe;
// NCO→DSP pipeline registers — breaks the long NCO sin/cos → DSP48E1 B-port route
// DONT_TOUCH prevents Vivado from absorbing these into the DSP or optimizing away
(* DONT_TOUCH = "TRUE" *) reg signed [15:0] cos_nco_pipe, sin_nco_pipe;
// Post-DSP retiming registers — breaks DSP48E1 CLK→P to fabric timing path
// This extra pipeline stage absorbs the 1.866ns DSP output prop delay + routing,
// ensuring WNS > 0 at 400 MHz regardless of placement seed
(* DONT_TOUCH = "TRUE" *) reg signed [MIXER_WIDTH+NCO_WIDTH-1:0] mult_i_retimed, mult_q_retimed;

// Output stage registers
reg signed [17:0] baseband_i_reg, baseband_q_reg;
reg baseband_valid_reg;

// ============================================================================
// Phase Dithering Signals
// ============================================================================
wire [7:0] phase_dither_bits;
reg [31:0] phase_inc_dithered;

// ============================================================================
// Debug Signal Assignments
// ============================================================================
assign debug_internal_i = mixed_i[25:8];
assign debug_internal_q = mixed_q[25:8];
assign debug_sample_count = sample_counter;
assign debug_mixed_i_trunc = mixed_i[25:8];
assign debug_mixed_q_trunc = mixed_q[25:8];

// ============================================================================
// 400 MHz Reset Synchronizer
//
// reset_n arrives from the 100 MHz domain (sys_reset_n from radar_system_top).
// Using it directly as an async reset in the 400 MHz domain causes the reset
// deassertion edge to violate timing: the 100 MHz flip-flop driving reset_n
// has its output fanning out to 1156 registers across the FPGA in the 400 MHz
// domain, requiring 18.243ns of routing (WNS = -18.081ns).
//
// Solution: 2-stage async-assert, sync-deassert reset synchronizer in the
// 400 MHz domain. Reset assertion is immediate (asynchronous — combinatorial
// path from reset_n to all 400 MHz registers). Reset deassertion is
//
// reset_400m   : ACTIVE-HIGH registered reset with (* max_fanout = 50 *).
//                This is THE signal fed to every synchronous 400 MHz FDRE
//                and every DSP48E1 RST pin in this module and its children
//                (NCO, CIC, LFSR). Vivado replicates the register (~14
//                copies) so each replica drives ≈50 loads regionally,
//                eliminating the single-LUT1 / 702-load net that caused
//                WNS=-0.626 ns in Build N.
//
// System-level invariants preserved:
//   I1  Reset assertion propagates to all 400 MHz regs within ≤3 clk edges
//       (2 sync + 1 replicated-reg fanout).  At 400 MHz = 7.5 ns << any
//       system-level reset assertion duration.
//   I2  Reset de-assertion is always synchronous to clk_400m (via
//       reset_sync_400m), never glitches.
//   I3  DSP48E1 RST pins are all fed from Q of a register — glitch-free.
//   I4  No new CDC introduced: reset_400m is entirely in clk_400m domain.
//   I5  Power-up: reset_n is asserted externally and mmcm_locked is low;
//       reset_sync_400m stays 2'b00, reset_400m stays 1'b1, downstream
//       FDREs stay cleared. Safe.
// ============================================================================
(* ASYNC_REG = "TRUE" *) reg [1:0] reset_sync_400m = 2'b00;
(* max_fanout = 50 *) wire reset_n_400m = reset_sync_400m[1];

// Active-high replicated reset for all synchronous 400 MHz consumers
(* max_fanout = 50 *) reg reset_400m = 1'b1;

always @(posedge clk_400m or negedge reset_n) begin
    if (!reset_n) begin
        reset_sync_400m <= 2'b00;
        reset_400m      <= 1'b1;
    end else begin
        reset_sync_400m <= {reset_sync_400m[0], 1'b1};
        reset_400m      <= ~reset_sync_400m[1];
    end
end

// CDC synchronization for control signals (2-stage synchronizers)
(* ASYNC_REG = "TRUE" *) reg [1:0] mixers_enable_sync_chain;
(* ASYNC_REG = "TRUE" *) reg [1:0] force_saturation_sync_chain;
wire mixers_enable_sync;
wire force_saturation_sync;
assign mixers_enable_sync = mixers_enable_sync_chain[1];
assign force_saturation_sync = force_saturation_sync_chain[1];

// Sync reset via reset_400m (replicated, max_fanout=50). Was async on
// reset_n_400m — see "400 MHz RESET DISTRIBUTION" comment above.
always @(posedge clk_400m) begin
    if (reset_400m) begin
        mixers_enable_sync_chain <= 2'b00;
        force_saturation_sync_chain <= 2'b00;
    end else begin
        mixers_enable_sync_chain <= {mixers_enable_sync_chain[0], mixers_enable};
        force_saturation_sync_chain <= {force_saturation_sync_chain[0], force_saturation};
    end
end

// ============================================================================
// Sample Counter and Debug Monitoring
// ============================================================================
always @(posedge clk_400m) begin
    if (reset_400m || reset_monitors) begin
        sample_counter <= 0;
        error_counter <= 0;
    end else if (adc_data_valid_i && adc_data_valid_q ) begin
        sample_counter <= sample_counter + 1;
    end
end


// ============================================================================
// Enhanced Phase Dithering Instance
// ============================================================================
lfsr_dither_enhanced #(
    .DITHER_WIDTH(8)
) phase_dither_gen (
    .clk(clk_400m),
    .reset_n(reset_n_400m),
    .enable(nco_ready),
    .dither_out(phase_dither_bits)
);

// ============================================================================
// Phase Increment Calculation with Dithering
// ============================================================================
// Calculate phase increment for 120MHz IF at 400MHz sampling
localparam PHASE_INC_120MHZ = 32'h4CCCCCCD;

// Apply dithering to reduce spurious tones (registered for 400 MHz timing)
always @(posedge clk_400m) begin
    if (reset_400m)
        phase_inc_dithered <= PHASE_INC_120MHZ;
    else
        phase_inc_dithered <= PHASE_INC_120MHZ + {24'b0, phase_dither_bits};
end

// ============================================================================
// Enhanced NCO with Diagnostics
// ============================================================================
nco_400m_enhanced nco_core (
    .clk_400m(clk_400m),
    .reset_n(reset_n_400m),
    .frequency_tuning_word(phase_inc_dithered),
    .phase_valid(mixers_enable),
    .phase_offset(16'h0000),
    .sin_out(sin_out),
    .cos_out(cos_out),
    .dds_ready(nco_ready)
);

// ============================================================================
// Enhanced Mixing Stage — DSP48E1 direct instantiation for 400 MHz timing
//
// Architecture:
//   ADC data → sign-extend to 18b → DSP48E1 A-port (AREG=1 pipelines it)
//   NCO cos/sin → fabric pipeline reg → DSP48E1 B-port (BREG=1 pipelines it)
//   Multiply result captured by MREG=1, then output registered by PREG=1
//   force_saturation override applied AFTER DSP48E1 output (not on input path)
//
// Latency: 4 clock cycles (1 NCO pipe + 1 AREG/BREG + 1 MREG + 1 PREG) + 1 retiming = 5 total
// PREG=1 absorbs DSP48E1 CLK→P delay internally, preventing fabric timing violations
// In simulation (Icarus), uses behavioral equivalent since DSP48E1 is Xilinx-only
// ============================================================================

// Combinational ADC sign conversion (no register — DSP48E1 AREG handles it)
assign adc_signed_w = {1'b0, adc_data, {(MIXER_WIDTH-ADC_WIDTH-1){1'b0}}} - 
                      {1'b0, {ADC_WIDTH{1'b1}}, {(MIXER_WIDTH-ADC_WIDTH-1){1'b0}}} / 2;

// Valid pipeline: 5-stage shift register (1 NCO pipe + 3 DSP48E1 AREG+MREG+PREG + 1 retiming)
always @(posedge clk_400m) begin
    if (reset_400m) begin
        dsp_valid_pipe <= 5'b00000;
    end else begin
        dsp_valid_pipe <= {dsp_valid_pipe[3:0], (nco_ready && adc_data_valid_i && adc_data_valid_q)};
    end
end

`ifdef SIMULATION
// ---- Behavioral model for Icarus Verilog simulation ----
// Mimics NCO pipeline + DSP48E1 with AREG=1, BREG=1, MREG=1, PREG=1 (4-cycle DSP + 1 NCO pipe)
reg signed [MIXER_WIDTH-1:0] adc_signed_reg;     // Models AREG
reg signed [15:0] cos_pipe_reg, sin_pipe_reg;     // Models BREG
reg signed [MIXER_WIDTH+NCO_WIDTH-1:0] mult_i_internal, mult_q_internal;  // Models MREG
reg signed [MIXER_WIDTH+NCO_WIDTH-1:0] mult_i_reg, mult_q_reg;            // Models PREG

// Stage 0: NCO pipeline — breaks long NCO→DSP route (matches synthesis fabric registers)
always @(posedge clk_400m) begin
    if (reset_400m) begin
        cos_nco_pipe <= 0;
        sin_nco_pipe <= 0;
    end else begin
        cos_nco_pipe <= cos_out;
        sin_nco_pipe <= sin_out;
    end
end

// Stage 1: AREG/BREG equivalent (uses pipelined NCO outputs)
always @(posedge clk_400m) begin
    if (reset_400m) begin
        adc_signed_reg <= 0;
        cos_pipe_reg <= 0;
        sin_pipe_reg <= 0;
    end else begin
        adc_signed_reg <= adc_signed_w;
        cos_pipe_reg <= cos_nco_pipe;
        sin_pipe_reg <= sin_nco_pipe;
    end
end

// Stage 2: MREG equivalent
always @(posedge clk_400m) begin
    if (reset_400m) begin
        mult_i_internal <= 0;
        mult_q_internal <= 0;
    end else begin
        mult_i_internal <= $signed(adc_signed_reg) * $signed(cos_pipe_reg);
        mult_q_internal <= $signed(adc_signed_reg) * $signed(sin_pipe_reg);
    end
end

// Stage 3: PREG equivalent
always @(posedge clk_400m) begin
    if (reset_400m) begin
        mult_i_reg <= 0;
        mult_q_reg <= 0;
    end else begin
        mult_i_reg <= mult_i_internal;
        mult_q_reg <= mult_q_internal;
    end
end

// Stage 4: Post-DSP retiming register (matches synthesis path)
always @(posedge clk_400m) begin
    if (reset_400m) begin
        mult_i_retimed <= 0;
        mult_q_retimed <= 0;
    end else begin
        mult_i_retimed <= mult_i_reg;
        mult_q_retimed <= mult_q_reg;
    end
end

`else
// ---- Direct DSP48E1 instantiation for Vivado synthesis ----
// This guarantees AREG/BREG/MREG are used, achieving timing closure at 400 MHz
wire [47:0] dsp_p_i, dsp_p_q;

// NCO pipeline stage — breaks the long NCO sin/cos → DSP48E1 B-port route
// (1.505ns routing observed in Build 26). These fabric registers are placed
// near the DSP by the placer, splitting the route into two shorter segments.
// DONT_TOUCH on the reg declaration (above) prevents absorption/retiming.
always @(posedge clk_400m) begin
    if (reset_400m) begin
        cos_nco_pipe <= 0;
        sin_nco_pipe <= 0;
    end else begin
        cos_nco_pipe <= cos_out;
        sin_nco_pipe <= sin_out;
    end
end

// DSP48E1 for I-channel mixer (adc_signed * cos_out)
DSP48E1 #(
    // Feature control attributes
    .A_INPUT("DIRECT"),
    .B_INPUT("DIRECT"),
    .USE_DPORT("FALSE"),
    .USE_MULT("MULTIPLY"),
    .USE_SIMD("ONE48"),
    .AREG(1),
    .BREG(1),
    .MREG(1),
    .PREG(1),
    .ADREG(0),
    .ACASCREG(1),
    .BCASCREG(1),
    .ALUMODEREG(0),
    .CARRYINREG(0),
    .CARRYINSELREG(0),
    .CREG(0),
    .DREG(0),
    .INMODEREG(0),
    .OPMODEREG(0),
    .AUTORESET_PATDET("NO_RESET"),
    .MASK(48'h3fffffffffff),
    .PATTERN(48'h000000000000),
    .SEL_MASK("MASK"),
    .SEL_PATTERN("PATTERN"),
    .USE_PATTERN_DETECT("NO_PATDET")
) dsp_mixer_i (
    // Clock and reset
    .CLK(clk_400m),
    .RSTA(reset_400m),
    .RSTB(reset_400m),
    .RSTM(reset_400m),
    .RSTP(reset_400m),
    .RSTALLCARRYIN(1'b0),
    .RSTALUMODE(1'b0),
    .RSTCTRL(1'b0),
    .RSTC(1'b0),
    .RSTD(1'b0),
    .RSTINMODE(1'b0),
    // Clock enables
    .CEA1(1'b0),       // AREG=1 uses CEA2
    .CEA2(1'b1),
    .CEB1(1'b0),       // BREG=1 uses CEB2
    .CEB2(1'b1),
    .CEM(1'b1),
    .CEP(1'b1),         // P register clock enable (PREG=1)
    .CEAD(1'b0),
    .CEALUMODE(1'b0),
    .CECARRYIN(1'b0),
    .CECTRL(1'b0),
    .CEC(1'b0),
    .CED(1'b0),
    .CEINMODE(1'b0),
    // Data ports
    .A({{12{adc_signed_w[MIXER_WIDTH-1]}}, adc_signed_w}),   // Sign-extend 18b to 30b
    .B({{2{cos_nco_pipe[15]}}, cos_nco_pipe}),                // Sign-extend 16b to 18b (pipelined)
    .C(48'b0),
    .D(25'b0),
    .CARRYIN(1'b0),
    // Control ports
    .OPMODE(7'b0000101),       // P = M (multiply only, no accumulate)
    .ALUMODE(4'b0000),         // Z + X + Y + CIN
    .INMODE(5'b00000),         // A2 * B2 (direct)
    .CARRYINSEL(3'b000),
    // Output ports
    .P(dsp_p_i),
    .PATTERNDETECT(),
    .PATTERNBDETECT(),
    .OVERFLOW(),
    .UNDERFLOW(),
    .CARRYOUT(),
    // Cascade ports (unused)
    .ACIN(30'b0),
    .BCIN(18'b0),
    .CARRYCASCIN(1'b0),
    .MULTSIGNIN(1'b0),
    .PCIN(48'b0),
    .ACOUT(),
    .BCOUT(),
    .CARRYCASCOUT(),
    .MULTSIGNOUT(),
    .PCOUT()
);

// DSP48E1 for Q-channel mixer (adc_signed * sin_out)
DSP48E1 #(
    .A_INPUT("DIRECT"),
    .B_INPUT("DIRECT"),
    .USE_DPORT("FALSE"),
    .USE_MULT("MULTIPLY"),
    .USE_SIMD("ONE48"),
    .AREG(1),
    .BREG(1),
    .MREG(1),
    .PREG(1),
    .ADREG(0),
    .ACASCREG(1),
    .BCASCREG(1),
    .ALUMODEREG(0),
    .CARRYINREG(0),
    .CARRYINSELREG(0),
    .CREG(0),
    .DREG(0),
    .INMODEREG(0),
    .OPMODEREG(0),
    .AUTORESET_PATDET("NO_RESET"),
    .MASK(48'h3fffffffffff),
    .PATTERN(48'h000000000000),
    .SEL_MASK("MASK"),
    .SEL_PATTERN("PATTERN"),
    .USE_PATTERN_DETECT("NO_PATDET")
) dsp_mixer_q (
    .CLK(clk_400m),
    .RSTA(reset_400m),
    .RSTB(reset_400m),
    .RSTM(reset_400m),
    .RSTP(reset_400m),
    .RSTALLCARRYIN(1'b0),
    .RSTALUMODE(1'b0),
    .RSTCTRL(1'b0),
    .RSTC(1'b0),
    .RSTD(1'b0),
    .RSTINMODE(1'b0),
    .CEA1(1'b0),
    .CEA2(1'b1),
    .CEB1(1'b0),
    .CEB2(1'b1),
    .CEM(1'b1),
    .CEP(1'b1),         // P register clock enable (PREG=1)
    .CEAD(1'b0),
    .CEALUMODE(1'b0),
    .CECARRYIN(1'b0),
    .CECTRL(1'b0),
    .CEC(1'b0),
    .CED(1'b0),
    .CEINMODE(1'b0),
    .A({{12{adc_signed_w[MIXER_WIDTH-1]}}, adc_signed_w}),
    .B({{2{sin_nco_pipe[15]}}, sin_nco_pipe}),
    .C(48'b0),
    .D(25'b0),
    .CARRYIN(1'b0),
    .OPMODE(7'b0000101),
    .ALUMODE(4'b0000),
    .INMODE(5'b00000),
    .CARRYINSEL(3'b000),
    .P(dsp_p_q),
    .PATTERNDETECT(),
    .PATTERNBDETECT(),
    .OVERFLOW(),
    .UNDERFLOW(),
    .CARRYOUT(),
    .ACIN(30'b0),
    .BCIN(18'b0),
    .CARRYCASCIN(1'b0),
    .MULTSIGNIN(1'b0),
    .PCIN(48'b0),
    .ACOUT(),
    .BCOUT(),
    .CARRYCASCOUT(),
    .MULTSIGNOUT(),
    .PCOUT()
);

// Extract the multiply result from DSP48E1 P output
// adc_signed is 18 bits, NCO is 16 bits → product is 34 bits (bits [33:0] of P)
wire signed [MIXER_WIDTH+NCO_WIDTH-1:0] mult_i_reg = dsp_p_i[MIXER_WIDTH+NCO_WIDTH-1:0];
wire signed [MIXER_WIDTH+NCO_WIDTH-1:0] mult_q_reg = dsp_p_q[MIXER_WIDTH+NCO_WIDTH-1:0];

// Stage 4: Post-DSP retiming register — breaks DSP48E1 CLK→P to fabric path
// Without this, the DSP output prop delay (1.866ns) + routing (0.515ns) exceeds
// the 2.500ns clock period at slow process corner
always @(posedge clk_400m) begin
    if (reset_400m) begin
        mult_i_retimed <= 0;
        mult_q_retimed <= 0;
    end else begin
        mult_i_retimed <= mult_i_reg;
        mult_q_retimed <= mult_q_reg;
    end
end

`endif

// ============================================================================
// Post-DSP48E1 output stage: force_saturation override + overflow detection
// force_saturation mux is intentionally AFTER the DSP48E1 output to avoid
// polluting the critical input path with extra logic
// ============================================================================
always @(posedge clk_400m) begin
    if (reset_400m) begin
        mixed_i <= 0;
        mixed_q <= 0;
        mixed_valid <= 0;
        mixer_overflow_i <= 0;
        mixer_overflow_q <= 0;
        saturation_count <= 0;
        overflow_detected <= 0;
    end else if (dsp_valid_pipe[4]) begin
        // Force saturation for testing (applied after DSP output, not on input path)
        if (force_saturation_sync) begin
            mixed_i <= 34'h1FFFFFFFF;
            mixed_q <= 34'h200000000;
            mixer_overflow_i <= 1'b1;
            mixer_overflow_q <= 1'b1;
        end else begin
            // Normal path: take retimed DSP48E1 multiply result
            mixed_i <= mult_i_retimed;
            mixed_q <= mult_q_retimed;
            
            // Overflow detection on retimed multiply result
            mixer_overflow_i <= (mult_i_retimed > (2**(MIXER_WIDTH+NCO_WIDTH-2)-1)) || 
                               (mult_i_retimed < -(2**(MIXER_WIDTH+NCO_WIDTH-2)));
            mixer_overflow_q <= (mult_q_retimed > (2**(MIXER_WIDTH+NCO_WIDTH-2)-1)) || 
                               (mult_q_retimed < -(2**(MIXER_WIDTH+NCO_WIDTH-2)));
        end
        
        mixed_valid <= 1;
        
        if (mixer_overflow_i || mixer_overflow_q) begin
            saturation_count <= saturation_count + 1;
            overflow_detected <= 1'b1;
        end else begin
            overflow_detected <= 1'b0;
        end
        
    end else begin
        mixed_valid <= 0;
        mixer_overflow_i <= 0;
        mixer_overflow_q <= 0;
        overflow_detected <= 1'b0;
    end
end

// ============================================================================
// Enhanced CIC Decimators
// ============================================================================
wire cic_valid_i, cic_valid_q;

cic_decimator_4x_enhanced cic_i_inst (
    .clk(clk_400m),
    .reset_n(reset_n_400m),
    .data_in(mixed_i[33:16]),
    .data_valid(mixed_valid),
    .data_out(cic_i_out),
    .data_out_valid(cic_valid_i)
);

cic_decimator_4x_enhanced cic_q_inst (
    .clk(clk_400m),
    .reset_n(reset_n_400m),
    .data_in(mixed_q[33:16]),
    .data_valid(mixed_valid),
    .data_out(cic_q_out),
    .data_out_valid(cic_valid_q)
);

assign cic_valid = cic_valid_i & cic_valid_q;

// ============================================================================
// Enhanced FIR Filters with FIXED valid signal handling
// NOTE: Wire declarations moved BEFORE CDC instances to fix forward-reference
//       error in Icarus Verilog (was originally after CDC instantiation)
// ============================================================================
wire fir_in_valid_i, fir_in_valid_q;
wire fir_valid_i, fir_valid_q;
wire fir_i_ready, fir_q_ready;
wire [17:0] fir_d_in_i, fir_d_in_q; 

cdc_adc_to_processing #(
    .WIDTH(18),
    .STAGES(3)
)CDC_FIR_i(
    .src_clk(clk_400m),
    .dst_clk(clk_100m),
    .src_reset_n(reset_n_400m),
    .dst_reset_n(reset_n),
    .src_data(cic_i_out),
    .src_valid(cic_valid_i),
    .dst_data(fir_d_in_i),
    .dst_valid(fir_in_valid_i)
);

cdc_adc_to_processing #(
    .WIDTH(18),
    .STAGES(3)
)CDC_FIR_q(
    .src_clk(clk_400m),
    .dst_clk(clk_100m),
    .src_reset_n(reset_n_400m),
    .dst_reset_n(reset_n),
    .src_data(cic_q_out),
    .src_valid(cic_valid_q),
    .dst_data(fir_d_in_q),
    .dst_valid(fir_in_valid_q)
);

// ============================================================================
// FIR Filter Instances
// ============================================================================

// FIR I channel
fir_lowpass_parallel_enhanced fir_i_inst (
    .clk(clk_100m),
    .reset_n(reset_n),
    .data_in(fir_d_in_i),  // Use synchronized data
    .data_valid(fir_in_valid_i),  // Use synchronized valid
    .data_out(fir_i_out),
    .data_out_valid(fir_valid_i),
    .fir_ready(fir_i_ready),
    .filter_overflow()
);

// FIR Q channel  
fir_lowpass_parallel_enhanced fir_q_inst (
    .clk(clk_100m),
    .reset_n(reset_n),
    .data_in(fir_d_in_q),  // Use synchronized data
    .data_valid(fir_in_valid_q),  // Use synchronized valid
    .data_out(fir_q_out),
    .data_out_valid(fir_valid_q),
    .fir_ready(fir_q_ready),
    .filter_overflow()
);

assign fir_valid = fir_valid_i & fir_valid_q;

// ============================================================================
// Enhanced Output Stage
// ============================================================================
always @(posedge clk_100m or negedge reset_n) begin
    if (!reset_n) begin
        baseband_i_reg <= 0;
        baseband_q_reg <= 0;
        baseband_valid_reg <= 0;
    end else if (fir_valid) begin
        baseband_i_reg <= fir_i_out;
        baseband_q_reg <= fir_q_out;
        baseband_valid_reg <= 1;
    end else begin
        baseband_valid_reg <= 0;
    end
end


// ============================================================================
// Output Assignments
// ============================================================================
assign baseband_i = baseband_i_reg;
assign baseband_q = baseband_q_reg;
assign baseband_valid_i = baseband_valid_reg;
assign baseband_valid_q = baseband_valid_reg;
assign ddc_status = {mixer_overflow_i | mixer_overflow_q, nco_ready};
assign mixer_saturation = overflow_detected;
assign ddc_diagnostics = {saturation_count, error_counter[4:0]};

// ============================================================================
// Enhanced Debug and Monitoring
// ============================================================================
reg [31:0] debug_cic_count, debug_fir_count, debug_bb_count;

`ifdef SIMULATION
always @(posedge clk_100m) begin
    
    if (fir_valid_i && debug_fir_count < 20) begin
        debug_fir_count <= debug_fir_count + 1;
        $display("FIR_OUTPUT: fir_i=%6d, fir_q=%6d", fir_i_out, fir_q_out);
    end
    
    if (adc_data_valid_i && adc_data_valid_q && debug_bb_count < 20) begin
        debug_bb_count <= debug_bb_count + 1;
        $display("BASEBAND_OUT: i=%6d, q=%6d, count=%0d", 
                 baseband_i, baseband_q, debug_bb_count);
    end
end
`endif

// In ddc_400m.v, add these debug signals:

// Debug monitoring (simulation only)
`ifdef SIMULATION
reg [31:0] debug_adc_count = 0;
reg [31:0] debug_baseband_count = 0;

always @(posedge clk_400m) begin
    if (adc_data_valid_i && adc_data_valid_q && debug_adc_count < 10) begin
        debug_adc_count <= debug_adc_count + 1;
        $display("DDC_ADC: data=%0d, count=%0d, time=%t", 
                 adc_data, debug_adc_count, $time);
    end
end

always @(posedge clk_100m) begin
    if (baseband_valid_i && baseband_valid_q && debug_baseband_count < 10) begin
        debug_baseband_count <= debug_baseband_count + 1;
        $display("DDC_BASEBAND: i=%0d, q=%0d, count=%0d, time=%t", 
                 baseband_i, baseband_q, debug_baseband_count, $time);
    end
end
`endif


endmodule

// ============================================================================
// Enhanced Phase Dithering Module
// ============================================================================
`timescale 1ns / 1ps

module lfsr_dither_enhanced #(
    parameter DITHER_WIDTH = 8  // Increased for better dithering
)(
    input wire clk,
    input wire reset_n,
    input wire enable,
    output wire [DITHER_WIDTH-1:0] dither_out
);

reg [DITHER_WIDTH-1:0] lfsr_reg;
reg [15:0] cycle_counter;
reg lock_detected;

// Polynomial for better randomness: x^8 + x^6 + x^5 + x^4 + 1
wire feedback;

generate
    if (DITHER_WIDTH == 4) begin
        assign feedback = lfsr_reg[3] ^ lfsr_reg[2];
    end else if (DITHER_WIDTH == 8) begin
        assign feedback = lfsr_reg[7] ^ lfsr_reg[5] ^ lfsr_reg[4] ^ lfsr_reg[3];
    end else begin
        assign feedback = lfsr_reg[DITHER_WIDTH-1] ^ lfsr_reg[DITHER_WIDTH-2];
    end
endgenerate

// ============================================================================
// RESET FAN-OUT INVARIANT: registered active-high reset with max_fanout=50.
// See cic_decimator_4x_enhanced.v for full reasoning. reset_n here is driven
// by the parent DDC's reset_n_400m (already synchronized to clk_400m), so
// sync reset on the LFSR is safe. INIT=1'b1 holds LFSR in reset on power-up.
// ============================================================================
(* max_fanout = 50 *) reg reset_h = 1'b1;
always @(posedge clk) reset_h <= ~reset_n;

always @(posedge clk) begin
    if (reset_h) begin
        lfsr_reg <= {DITHER_WIDTH{1'b1}};  // Non-zero initial state
        cycle_counter <= 0;
        lock_detected <= 0;
    end else if (enable) begin
        lfsr_reg <= {lfsr_reg[DITHER_WIDTH-2:0], feedback};
        cycle_counter <= cycle_counter + 1;
        
        // Detect LFSR lock after sufficient cycles
        if (cycle_counter > (2**DITHER_WIDTH * 8)) begin
            lock_detected <= 1'b1;
        end
    end
end

assign dither_out = lfsr_reg;

endmodule
