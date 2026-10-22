`timescale 1ns / 1ps

module nco_400m_enhanced (
    input wire clk_400m,
    input wire reset_n,
    input wire [31:0] frequency_tuning_word,
    input wire phase_valid,
    input wire [15:0] phase_offset,
    output reg signed [15:0] sin_out,
    output reg signed [15:0] cos_out,
    output reg dds_ready
);

// ============================================================================
// 6-stage pipelined NCO for 400 MHz timing closure
//
// Stage 1: Phase accumulator update (DSP48E1 in P=P+C mode)
//          DSP48E1 does: P_reg <= P_reg + C_port (frequency_tuning_word)
//          The P register output IS the phase accumulator — no CARRY4 chain.
//          phase_accum_reg <= P_output[31:0] (fabric register captures DSP output)
// Stage 2: Offset addition in fabric (registered)
//          phase_with_offset <= phase_accum_reg + {phase_offset, 16'b0}
//          Breaking DSP→CARRY4 into two registered stages eliminates the
//          critical path (was -0.594ns WNS in Build 6)
// Stage 3a: Register LUT address (lut_index) and quadrant from phase_with_offset
//           Only 2 registers driven (minimal fanout, short routes)
// Stage 3b: LUT read using registered lut_index → register abs values + quadrant
//           Registered LUT address → combinational LUT6 read → register
//           Eliminates the routing-dominant critical path (-0.100ns in Build 8)
// Stage 4: Compute negations from registered abs values → register neg values
//          (CARRY4 x4 chain has registered inputs, fits in 2.5ns easily)
// Stage 5: Quadrant sign application → sin_out, cos_out (pure MUX, no arith)
//
// Total latency: 6 cycles from phase_valid to sin/cos output
// Max logic levels per stage: Stage 1=DSP48E1(internal), Stage 2=4(CARRY4x5),
//   Stage 3a=1(LUT3 quadrant+index decode), Stage 3b=1(LUT6 ROM read),
//   Stage 4=4(CARRY4 chain), Stage 5=1(MUX)
// ============================================================================

// Phase accumulator — DSP48E1 P output provides the accumulated phase
// In simulation: behavioral reg. In synthesis: DSP48E1 P[31:0].
reg [31:0] phase_accum_reg;     // Stage 1 output: registered DSP48E1 P[31:0]
reg [31:0] phase_with_offset;   // Stage 2 output: phase_accum_reg + offset

// Stage 3a pipeline registers: registered LUT address + quadrant
reg [5:0] lut_index_pipe_sin;
reg [5:0] lut_index_pipe_cos;
reg [1:0] quadrant_pipe;

// Stage 3b pipeline registers: LUT output + quadrant
reg [15:0] sin_abs_reg, cos_abs_reg;
reg [1:0] quadrant_reg;

// Stage 4 pipeline registers: pre-computed negations + abs copies + quadrant
reg signed [15:0] sin_neg_reg, cos_neg_reg;
reg [15:0] sin_abs_reg2, cos_abs_reg2;  // Pass-through for Stage 5 MUX
reg [1:0] quadrant_reg2;                 // Pass-through for Stage 5 MUX

// Valid pipeline: tracks 6-stage latency
reg [5:0] valid_pipe;

// ============================================================================
// RESET FAN-OUT INVARIANT (Build N+1 fix for WNS=-0.626ns at 400 MHz):
// ============================================================================
// reset_h is an ACTIVE-HIGH, REGISTERED copy of ~reset_n with (* max_fanout=50 *).
// Vivado replicates this register (14+ copies) so each copy drives ≈50 loads
// regionally, avoiding the single-LUT1 / 702-load net that caused timing
// failure in Build N. It feeds:
//   - DSP48E1 RSTP/RSTC on the phase-accumulator DSP (below)
//   - All pipeline-stage fabric FDREs (synchronous reset)
// Invariants (see cic_decimator_4x_enhanced.v for full reasoning):
//   I1 correctness:      reset_h == ~reset_n one cycle later
//   I2 glitch-free:      registered output
//   I3 power-up safe:    INIT=1'b1 holds all downstream in reset until first
//                        valid clock edge; reset_n is low on power-up anyway
//   I4 de-assert lat.:   +1 cycle vs. direct async; negligible at 400 MHz
// ============================================================================
(* max_fanout = 50 *) reg reset_h = 1'b1;
always @(posedge clk_400m) reset_h <= ~reset_n;

// Use only the top 8 bits for LUT addressing (256-entry LUT equivalent)
wire [7:0] lut_address = phase_with_offset[31:24];

// Quarter-wave sine LUT (0-90 degrees only)
// Force distributed RAM (LUTRAM) — the 64x16 LUT is only 1024 bits, far too
// small for BRAM. BRAM CLK→DOADO delay (2.454ns) + downstream negation logic
// (1.236ns) exceeded the 2.5ns period at 400 MHz (WNS = -2.238ns). LUTRAM
// read is combinatorial (~0.5ns through LUTs), giving the Stage 2→3 negation
// path ~2.1ns of budget which fits comfortably.
(* ram_style = "distributed" *) reg [15:0] sin_lut [0:63]; // 64 entries for 0-90 degrees

// Initialize sine LUT
integer lut_init_i;
initial begin
    for (lut_init_i = 0; lut_init_i < 64; lut_init_i = lut_init_i + 1) begin
        sin_lut[lut_init_i] = 16'h0000;
    end
    
    // Initialize quarter-wave sine LUT (0-90 degrees)
    // LUT[k] = round(32767 * sin(pi/2 * k / 64)), monotonically increasing
    // FIX: Original LUT peaked at index 42 then decreased — broke cos=sin[63-k] quadrature
    sin_lut[0] = 16'h0000; sin_lut[1] = 16'h0324; sin_lut[2] = 16'h0648; sin_lut[3] = 16'h096A;
    sin_lut[4] = 16'h0C8C; sin_lut[5] = 16'h0FAB; sin_lut[6] = 16'h12C8; sin_lut[7] = 16'h15E2;
    sin_lut[8] = 16'h18F9; sin_lut[9] = 16'h1C0B; sin_lut[10] = 16'h1F1A; sin_lut[11] = 16'h2223;
    sin_lut[12] = 16'h2528; sin_lut[13] = 16'h2826; sin_lut[14] = 16'h2B1F; sin_lut[15] = 16'h2E11;
    sin_lut[16] = 16'h30FB; sin_lut[17] = 16'h33DF; sin_lut[18] = 16'h36BA; sin_lut[19] = 16'h398C;
    sin_lut[20] = 16'h3C56; sin_lut[21] = 16'h3F17; sin_lut[22] = 16'h41CE; sin_lut[23] = 16'h447A;
    sin_lut[24] = 16'h471C; sin_lut[25] = 16'h49B4; sin_lut[26] = 16'h4C3F; sin_lut[27] = 16'h4EBF;
    sin_lut[28] = 16'h5133; sin_lut[29] = 16'h539B; sin_lut[30] = 16'h55F5; sin_lut[31] = 16'h5842;
    sin_lut[32] = 16'h5A82; sin_lut[33] = 16'h5CB3; sin_lut[34] = 16'h5ED7; sin_lut[35] = 16'h60EB;
    sin_lut[36] = 16'h62F1; sin_lut[37] = 16'h64E8; sin_lut[38] = 16'h66CF; sin_lut[39] = 16'h68A6;
    sin_lut[40] = 16'h6A6D; sin_lut[41] = 16'h6C23; sin_lut[42] = 16'h6DC9; sin_lut[43] = 16'h6F5E;
    sin_lut[44] = 16'h70E2; sin_lut[45] = 16'h7254; sin_lut[46] = 16'h73B5; sin_lut[47] = 16'h7504;
    sin_lut[48] = 16'h7641; sin_lut[49] = 16'h776B; sin_lut[50] = 16'h7884; sin_lut[51] = 16'h7989;
    sin_lut[52] = 16'h7A7C; sin_lut[53] = 16'h7B5C; sin_lut[54] = 16'h7C29; sin_lut[55] = 16'h7CE3;
    sin_lut[56] = 16'h7D89; sin_lut[57] = 16'h7E1D; sin_lut[58] = 16'h7E9C; sin_lut[59] = 16'h7F09;
    sin_lut[60] = 16'h7F61; sin_lut[61] = 16'h7FA6; sin_lut[62] = 16'h7FD8; sin_lut[63] = 16'h7FF5;
end

// Combinational: quadrant determination and LUT index (feeds Stage 3a registers)
wire [1:0] quadrant_w = lut_address[7:6];
wire [5:0] lut_index = (quadrant_w[0] ^ quadrant_w[1]) ? ~lut_address[5:0] : lut_address[5:0];

// Combinational LUT read using REGISTERED lut_index_pipe (feeds Stage 3b registers)
// These wires are driven by lut_index_pipe (registered in Stage 3a), so the
// combinational path is just: lut_index_pipe_reg → LUT6 (distributed RAM read)
// This eliminates the LUT3→LUT6 two-level critical path from Build 8.
wire [15:0] sin_abs_w = sin_lut[lut_index_pipe_sin];
wire [15:0] cos_abs_w = sin_lut[63 - lut_index_pipe_cos];

// ============================================================================
// Stage 1: Phase accumulator (DSP48E1) — accumulates FTW each cycle
// Stage 2: Offset addition in fabric — breaks DSP→CARRY4 critical path
//
// The phase accumulator is the critical path bottleneck: a 32-bit addition
// requires 8 CARRY4 stages in fabric (2.826 ns > 2.5 ns budget at 400 MHz).
// Solution: Use DSP48E1 in P = P + C accumulate mode.
//   - C-port carries frequency_tuning_word (zero-extended to 48 bits)
//   - CREG=1 registers the tuning word inside the DSP
//   - PREG=1 registers the accumulator output (P = P + C each cycle)
//   - The DSP48E1 48-bit ALU performs the add internally at full speed
//   - Only P[31:0] is used (32-bit phase accumulator)
//
// Phase offset addition is split into a separate pipeline stage:
//   Stage 1: phase_accum_reg <= P[31:0]  (just capture the DSP output)
//   Stage 2: phase_with_offset <= phase_accum_reg + {phase_offset, 16'b0}
// This eliminates the DSP48E1.P→CARRY4 chain critical path (-0.594ns in Build 6).
// ============================================================================

`ifdef SIMULATION
// ---- Behavioral model for Icarus Verilog simulation ----
// Mimics DSP48E1 accumulator: P <= P + C, with CREG=1, PREG=1
// Stage 1: phase_accum_reg captures accumulator output
// Stage 2: phase_with_offset adds phase offset
reg [31:0] phase_accumulator;

always @(posedge clk_400m) begin
    if (reset_h) begin
        phase_accumulator <= 32'h00000000;
        phase_accum_reg   <= 32'h00000000;
        phase_with_offset <= 32'h00000000;
    end else if (phase_valid) begin
        // Stage 1: accumulate + capture
        phase_accumulator <= phase_accumulator + frequency_tuning_word;
        phase_accum_reg   <= phase_accumulator;
        // Stage 2: offset addition (uses previous cycle's phase_accum_reg)
        phase_with_offset <= phase_accum_reg + {phase_offset, 16'b0};
    end
end

`else
// ---- DSP48E1 phase accumulator for Vivado synthesis ----
// P = P + C mode: accumulates frequency_tuning_word each clock cycle
// Uses 1 DSP48E1 (total design: 5 of 240 available = 2.08%)
wire [47:0] phase_accum_p;          // DSP48E1 P output (48 bits, use [31:0])

DSP48E1 #(
    // Feature control
    .A_INPUT("DIRECT"),
    .B_INPUT("DIRECT"),
    .USE_DPORT("FALSE"),
    .USE_MULT("NONE"),           // No multiplier — pure ALU accumulate
    .USE_SIMD("ONE48"),
    // Pipeline registers
    .AREG(0),                    // A-port unused for accumulate
    .BREG(0),                    // B-port unused for accumulate
    .CREG(1),                    // Register frequency_tuning_word on C-port
    .MREG(0),                    // No multiplier
    .PREG(1),                    // P register IS the phase accumulator
    .ADREG(0),
    .ACASCREG(0),
    .BCASCREG(0),
    .ALUMODEREG(0),
    .CARRYINREG(0),
    .CARRYINSELREG(0),
    .DREG(0),
    .INMODEREG(0),
    .OPMODEREG(0),
    // Pattern detector (unused)
    .AUTORESET_PATDET("NO_RESET"),
    .MASK(48'h3fffffffffff),
    .PATTERN(48'h000000000000),
    .SEL_MASK("MASK"),
    .SEL_PATTERN("PATTERN"),
    .USE_PATTERN_DETECT("NO_PATDET")
) dsp_phase_accum (
    // Clock and reset
    .CLK(clk_400m),
    .RSTA(1'b0),
    .RSTB(1'b0),
    .RSTM(1'b0),
    .RSTP(reset_h),              // Reset P register (phase accumulator) — registered, max_fanout=50
    .RSTC(reset_h),              // Reset C register (tuning word)       — registered, max_fanout=50
    .RSTALLCARRYIN(1'b0),
    .RSTALUMODE(1'b0),
    .RSTCTRL(1'b0),
    .RSTD(1'b0),
    .RSTINMODE(1'b0),
    // Clock enables
    .CEA1(1'b0),
    .CEA2(1'b0),
    .CEB1(1'b0),
    .CEB2(1'b0),
    .CEC(1'b1),                  // Always register C (tuning word updates)
    .CEM(1'b0),
    .CEP(phase_valid),           // Only accumulate when phase_valid is asserted
    .CEAD(1'b0),
    .CEALUMODE(1'b0),
    .CECARRYIN(1'b0),
    .CECTRL(1'b0),
    .CED(1'b0),
    .CEINMODE(1'b0),
    // Data ports
    .A(30'b0),                   // Unused for P = P + C
    .B(18'b0),                   // Unused for P = P + C
    .C({16'b0, frequency_tuning_word}),  // Zero-extend 32-bit FTW to 48 bits
    .D(25'b0),
    .CARRYIN(1'b0),
    // Control ports
    .OPMODE(7'b0101100),         // Z=P (010), Y=C (11), X=0 (00) → P = P + C
    .ALUMODE(4'b0000),           // Z + X + Y + CIN (standard add)
    .INMODE(5'b00000),
    .CARRYINSEL(3'b000),
    // Output ports
    .P(phase_accum_p),
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

// Stage 1: Capture DSP48E1 P output into fabric register
// Stage 2: Add phase offset to captured value
// Split into two registered stages to break DSP48E1.P→CARRY4 critical path
always @(posedge clk_400m) begin
    if (reset_h) begin
        phase_accum_reg   <= 32'h00000000;
        phase_with_offset <= 32'h00000000;
    end else if (phase_valid) begin
        // Stage 1: just capture DSP output (no CARRY4 chain)
        phase_accum_reg   <= phase_accum_p[31:0];
        // Stage 2: offset add (CARRY4 chain from registered fabric→fabric, easy timing)
        phase_with_offset <= phase_accum_reg + {phase_offset, 16'b0};
    end
end

`endif

// ============================================================================
// Stage 3a: Register LUT address and quadrant from phase_with_offset
//           Only 2 registers driven (lut_index_pipe + quadrant_pipe)
//           Minimal fanout → short routes → easy timing
// ============================================================================
always @(posedge clk_400m) begin
    if (reset_h) begin
        lut_index_pipe_sin <= 6'b000000;
        lut_index_pipe_cos <= 6'b000000;
        quadrant_pipe      <= 2'b00;
    end else if (valid_pipe[1]) begin
        lut_index_pipe_sin <= lut_index;
        lut_index_pipe_cos <= lut_index;
        quadrant_pipe      <= quadrant_w;
    end
end

// ============================================================================
// Stage 3b: LUT read using registered lut_index_pipe + register abs values
//           Registered address → combinational LUT6 read → register
//           Only 1 logic level (LUT6), trivial timing
// ============================================================================
always @(posedge clk_400m) begin
    if (reset_h) begin
        sin_abs_reg <= 16'h0000;
        cos_abs_reg <= 16'h7FFF;
        quadrant_reg <= 2'b00;
    end else if (valid_pipe[2]) begin
        sin_abs_reg <= sin_abs_w;
        cos_abs_reg <= cos_abs_w;
        quadrant_reg <= quadrant_pipe;
    end
end

// ============================================================================
// Stage 4: Compute negations from registered abs values
//          CARRY4 x4 chain has registered inputs — easily fits in 2.5ns
//          Also pass through abs values and quadrant for Stage 5
// ============================================================================
always @(posedge clk_400m) begin
    if (reset_h) begin
        sin_neg_reg <= 16'h0000;
        cos_neg_reg <= -16'h7FFF;
        sin_abs_reg2 <= 16'h0000;
        cos_abs_reg2 <= 16'h7FFF;
        quadrant_reg2 <= 2'b00;
    end else if (valid_pipe[3]) begin
        sin_neg_reg <= -sin_abs_reg;
        cos_neg_reg <= -cos_abs_reg;
        sin_abs_reg2 <= sin_abs_reg;
        cos_abs_reg2 <= cos_abs_reg;
        quadrant_reg2 <= quadrant_reg;
    end
end

// ============================================================================
// Stage 5: Quadrant sign application → final sin/cos output
//          Uses pre-computed negated values from Stage 4 — pure MUX, no arithmetic
// ============================================================================
always @(posedge clk_400m) begin
    if (reset_h) begin
        sin_out <= 16'h0000;
        cos_out <= 16'h7FFF;
    end else if (valid_pipe[4]) begin
        case (quadrant_reg2)
            2'b00: begin // Quadrant I: sin+, cos+
                sin_out <= sin_abs_reg2;
                cos_out <= cos_abs_reg2;
            end
            2'b01: begin // Quadrant II: sin+, cos-
                sin_out <= sin_abs_reg2;
                cos_out <= cos_neg_reg;
            end
            2'b10: begin // Quadrant III: sin-, cos-
                sin_out <= sin_neg_reg;
                cos_out <= cos_neg_reg;
            end
            2'b11: begin // Quadrant IV: sin-, cos+
                sin_out <= sin_neg_reg;
                cos_out <= cos_abs_reg2;
            end
        endcase
    end
end

// ============================================================================
// Valid pipeline and dds_ready (6-stage latency)
// ============================================================================
always @(posedge clk_400m) begin
    if (reset_h) begin
        valid_pipe <= 6'b000000;
        dds_ready <= 1'b0;
    end else begin
        valid_pipe <= {valid_pipe[4:0], phase_valid};
        dds_ready <= valid_pipe[5];
    end
end

// Debug verification of LUT initialization (simulation only)
`ifdef SIMULATION
initial begin
    #10;
    $display("NCO: Sine LUT initialized with %0d entries", 64);
end
`endif

endmodule
