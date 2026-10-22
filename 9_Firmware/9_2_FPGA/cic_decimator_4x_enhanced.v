module cic_decimator_4x_enhanced (
    input wire clk,                 // 400MHz input clock
    input wire reset_n,
    input wire signed [17:0] data_in,  // 18-bit input
    input wire data_valid,
    output reg signed [17:0] data_out, // 18-bit output
    output reg data_out_valid,       // Valid at 100MHz
    // Enhanced monitoring outputs
    output reg saturation_detected,  // Latched saturation indicator
    output reg [7:0] max_value_monitor, // For gain control
    input wire reset_monitors        // Clear saturation and max value
);

parameter STAGES = 5;
parameter DECIMATION = 4;
parameter COMB_DELAY = 1;

// Accumulator width: DSP48E1 native 48-bit.
// CIC uses modular (wrapping) arithmetic so extra MSBs are harmless.
localparam ACC_WIDTH = 48;

// Comb section operates on 28-bit (18 + 5*log2(4) = 28, exact for comb range).
localparam COMB_WIDTH = 28;

// ============================================================================
// INTEGRATOR CHAIN — explicit DSP48E1 with PCOUT→PCIN cascade
// ============================================================================
// Integrator[0]: P = P + C,    C = sign_extend(data_in)  [from fabric]
// Integrator[k]: P = P + PCIN, PCIN from integrator[k-1] [dedicated cascade]
//
// The PCOUT→PCIN cascade uses dedicated silicon routing between vertically
// adjacent DSP48E1 tiles — zero fabric delay, guaranteed to meet 400+ MHz
// on 7-series regardless of speed grade.
//
// Active-high reset derived from reset_n (inverted and REGISTERED).
// CEP (clock enable for P register) gated by data_valid.
//
// ----------------------------------------------------------------------------
// RESET FAN-OUT INVARIANT (Build N+1 fix for WNS=-0.626ns at 400 MHz):
// ----------------------------------------------------------------------------
// Previously this was a combinational wire (`wire reset_h = ~reset_n`). Vivado
// collapsed all per-module inversions across the DDC hierarchy into a SINGLE
// shared LUT1, whose output fanned out to 702 loads (DSP48E1 RSTP/RSTB/RSTC
// plus FDRE R pins of all comb-stage DSP48E1s inferred via use_dsp="yes").
// Route delay alone on that net was 2.019–2.268 ns — nearly one full 2.5 ns
// period. Timing failed by 626 ps on the 400 MHz domain.
//
// Fix: convert reset_h to a REGISTERED signal with (* max_fanout = 50 *).
// Vivado treats max_fanout on a REG (not a wire) as authoritative and
// replicates the register into N copies, each placed near its ≈50 loads.
// Invariants preserved:
//   I1 (correctness):       reset_h is still active-high, equals ~reset_n
//                           after one clk edge; CIC reset is a RECEIVER-side
//                           synchronizer anyway (driven by reset_n_400m which
//                           is already sync'd in the parent DDC), so adding
//                           one more clk cycle of latency is safe.
//   I2 (glitch-free):       Registered output => inherently glitch-free,
//                           feeding DSP48E1 RST pins (which are synchronous
//                           to CLK, so they capture on the same edge anyway).
//   I3 (power-up safety):   reset_h is NOT async-reset itself. On power-up,
//                           FDRE INIT=0 starts reset_h LOW. First clk edge
//                           samples ~reset_n which is LOW on power-up (the
//                           parent DDC holds reset_n_400m low until the 2-
//                           stage synchronizer releases), so reset_h goes
//                           HIGH on cycle 1 and all DSPs see reset during
//                           the following cycles. System is held in reset
//                           for enough cycles that any initial register
//                           state garbage is overwritten. ✅
//   I4 (reset de-assertion):reset_h goes LOW one cycle AFTER reset_n_400m
//                           goes HIGH. Downstream DSPs come out of reset on
//                           the next clk edge after that. Total latency
//                           from system reset release to first valid sample:
//                           2 (sync chain) + 1 (reset_h reg) + 1 (first
//                           DSP output) = 4 cycles at 400 MHz = 10 ns.
//                           Negligible vs system reset assertion duration.
// ----------------------------------------------------------------------------
(* max_fanout = 50 *) reg reset_h = 1'b1;  // INIT=1'b1: registers start in reset state on power-up
always @(posedge clk) reset_h <= ~reset_n;

// Sign-extended input for integrator_0 C port (48-bit)
wire [ACC_WIDTH-1:0] data_in_c = {{(ACC_WIDTH-18){data_in[17]}}, data_in};

// DSP48E1 cascade wires
wire [47:0] pcout_0, pcout_1, pcout_2, pcout_3;
wire [47:0] p_out_0, p_out_1, p_out_2, p_out_3, p_out_4;

// Comb stage 0 DSP48E1 output wire (CREG+AREG+BREG pipelined subtract)
wire [47:0] comb_0_p_out;

// ============================================================================
// SHARED REGISTER DECLARATIONS
// ============================================================================
// These registers are referenced by both the synthesis DSP48E1 instances
// (inside `ifndef SIMULATION) and the behavioral simulation model (inside
// `else), as well as the shared fabric logic (after `endif).
// Icarus Verilog 13.0 requires registers to be declared before their first
// use within any `ifdef branch, so we declare them here — before the
// `ifndef SIMULATION block — rather than in the post-`endif shared section.
// ============================================================================
(* keep = "true", dont_touch = "true" *) reg signed [COMB_WIDTH-1:0] integrator_sampled;
(* keep = "true", dont_touch = "true", max_fanout = 1 *) reg signed [COMB_WIDTH-1:0] integrator_sampled_comb;
(* use_dsp = "yes" *) reg signed [COMB_WIDTH-1:0] comb [0:STAGES-1];
reg signed [COMB_WIDTH-1:0] comb_delay [0:STAGES-1][0:COMB_DELAY-1];

// Pipeline valid for comb stages 1-4: delayed by 1 cycle vs comb_pipe to
// account for CREG+AREG+BREG pipeline inside comb_0_dsp (explicit DSP48E1).
// Comb[0] result appears 1 cycle after data_valid_comb_pipe.
(* keep = "true", max_fanout = 16 *) reg data_valid_comb_0_out;

// Enhanced control and monitoring
reg [1:0] decimation_counter;
(* keep = "true", max_fanout = 16 *) reg data_valid_delayed;
(* keep = "true", max_fanout = 16 *) reg data_valid_comb;
(* keep = "true", max_fanout = 16 *) reg data_valid_comb_pipe;
reg [7:0] output_counter;
reg [ACC_WIDTH-1:0] max_integrator_value;
reg overflow_detected;
reg overflow_latched;

// Diagnostic registers
reg [7:0] saturation_event_count;
reg [31:0] sample_count;

// Comb-stage saturation flags
reg comb_overflow_latched;
reg comb_saturation_detected;
reg [7:0] comb_saturation_event_count;

// Temporary signals for calculations
reg signed [ACC_WIDTH-1:0] abs_integrator_value;
reg signed [COMB_WIDTH-1:0] temp_scaled_output;
reg signed [17:0] temp_output;

// Pipeline stage for saturation comparison
reg sat_pos;
reg sat_neg;
reg signed [17:0] temp_output_pipe;
reg data_out_valid_pipe;

integer i, j;

`ifndef SIMULATION
// ============================================================================
// SYNTHESIS: Explicit DSP48E1 instances with PCOUT→PCIN cascade
// ============================================================================

// --- Integrator 0: P = P + C (accumulate sign-extended input) ---
// OPMODE = 7'b0101100: Z=P(010), Y=C(11), X=0(00) → P = P + C
// CREG=1: C port is registered inside DSP48E1. This eliminates the
// fabric→DSP C-port setup timing violation (-0.415ns in Build 6).
// The CREG adds 1 cycle of latency before data reaches the ALU.
// CEC=data_valid gates the C register to match CEP behavior.
DSP48E1 #(
    .A_INPUT            ("DIRECT"),
    .B_INPUT            ("DIRECT"),
    .USE_DPORT          ("FALSE"),
    .USE_MULT           ("NONE"),
    .AUTORESET_PATDET   ("NO_RESET"),
    .MASK               (48'h3FFFFFFFFFFF),
    .PATTERN             (48'h000000000000),
    .SEL_MASK           ("MASK"),
    .SEL_PATTERN        ("PATTERN"),
    .USE_PATTERN_DETECT ("NO_PATDET"),
    .ACASCREG           (0),
    .ADREG              (0),
    .ALUMODEREG         (0),
    .AREG               (0),
    .BCASCREG           (0),
    .BREG               (0),
    .CARRYINREG         (0),
    .CARRYINSELREG      (0),
    .CREG               (1),       // C port registered inside DSP — eliminates fabric→DSP setup path
    .DREG               (0),
    .INMODEREG          (0),
    .MREG               (0),
    .OPMODEREG          (0),
    .PREG               (1)        // P register enabled (accumulator)
) integrator_0_dsp (
    .CLK                (clk),
    .A                  (30'd0),
    .B                  (18'd0),
    .C                  (data_in_c),
    .D                  (25'd0),
    .CARRYIN            (1'b0),
    .CARRYINSEL         (3'b000),
    .OPMODE             (7'b0101100),  // P = P + C
    .ALUMODE            (4'b0000),     // Z + (X + Y + CIN)
    .INMODE             (5'b00000),
    .CEA1               (1'b0),
    .CEA2               (1'b0),
    .CEB1               (1'b0),
    .CEB2               (1'b0),
    .CEC                (data_valid),  // Register C when data is valid (CREG=1)
    .CED                (1'b0),
    .CEM                (1'b0),
    .CEP                (data_valid),  // Accumulate only when data is valid
    .CEAD               (1'b0),
    .CEALUMODE          (1'b0),
    .CECTRL             (1'b0),
    .CECARRYIN          (1'b0),
    .CEINMODE           (1'b0),
    .RSTP               (reset_h),
    .RSTA               (1'b0),
    .RSTB               (1'b0),
    .RSTC               (reset_h),     // Reset C register (CREG=1) on reset
    .RSTD               (1'b0),
    .RSTM               (1'b0),
    .RSTALLCARRYIN      (1'b0),
    .RSTALUMODE         (1'b0),
    .RSTCTRL            (1'b0),
    .RSTINMODE          (1'b0),
    .P                  (p_out_0),
    .PCOUT              (pcout_0),
    .ACOUT              (),
    .BCOUT              (),
    .CARRYCASCOUT       (),
    .CARRYOUT           (),
    .MULTSIGNOUT        (),
    .OVERFLOW           (),
    .PATTERNBDETECT     (),
    .PATTERNDETECT      (),
    .UNDERFLOW          ()
);

// --- Integrator 1: P = P + PCIN (cascade from integrator_0) ---
// OPMODE = 7'b0010010: Z=PCIN(001), Y=0(00), X=P(10) → P = P + PCIN
DSP48E1 #(
    .A_INPUT            ("DIRECT"),
    .B_INPUT            ("DIRECT"),
    .USE_DPORT          ("FALSE"),
    .USE_MULT           ("NONE"),
    .AUTORESET_PATDET   ("NO_RESET"),
    .MASK               (48'h3FFFFFFFFFFF),
    .PATTERN             (48'h000000000000),
    .SEL_MASK           ("MASK"),
    .SEL_PATTERN        ("PATTERN"),
    .USE_PATTERN_DETECT ("NO_PATDET"),
    .ACASCREG           (0),
    .ADREG              (0),
    .ALUMODEREG         (0),
    .AREG               (0),
    .BCASCREG           (0),
    .BREG               (0),
    .CARRYINREG         (0),
    .CARRYINSELREG      (0),
    .CREG               (0),
    .DREG               (0),
    .INMODEREG          (0),
    .MREG               (0),
    .OPMODEREG          (0),
    .PREG               (1)
) integrator_1_dsp (
    .CLK                (clk),
    .A                  (30'd0),
    .B                  (18'd0),
    .C                  (48'd0),
    .D                  (25'd0),
    .PCIN               (pcout_0),
    .CARRYIN            (1'b0),
    .CARRYINSEL         (3'b000),
    .OPMODE             (7'b0010010),  // P = P + PCIN
    .ALUMODE            (4'b0000),
    .INMODE             (5'b00000),
    .CEA1               (1'b0),
    .CEA2               (1'b0),
    .CEB1               (1'b0),
    .CEB2               (1'b0),
    .CEC                (1'b0),
    .CED                (1'b0),
    .CEM                (1'b0),
    .CEP                (data_valid),
    .CEAD               (1'b0),
    .CEALUMODE          (1'b0),
    .CECTRL             (1'b0),
    .CECARRYIN          (1'b0),
    .CEINMODE           (1'b0),
    .RSTP               (reset_h),
    .RSTA               (1'b0),
    .RSTB               (1'b0),
    .RSTC               (1'b0),
    .RSTD               (1'b0),
    .RSTM               (1'b0),
    .RSTALLCARRYIN      (1'b0),
    .RSTALUMODE         (1'b0),
    .RSTCTRL            (1'b0),
    .RSTINMODE          (1'b0),
    .P                  (p_out_1),
    .PCOUT              (pcout_1),
    .ACOUT              (),
    .BCOUT              (),
    .CARRYCASCOUT       (),
    .CARRYOUT           (),
    .MULTSIGNOUT        (),
    .OVERFLOW           (),
    .PATTERNBDETECT     (),
    .PATTERNDETECT      (),
    .UNDERFLOW          ()
);

// --- Integrator 2: P = P + PCIN (cascade from integrator_1) ---
DSP48E1 #(
    .A_INPUT            ("DIRECT"),
    .B_INPUT            ("DIRECT"),
    .USE_DPORT          ("FALSE"),
    .USE_MULT           ("NONE"),
    .AUTORESET_PATDET   ("NO_RESET"),
    .MASK               (48'h3FFFFFFFFFFF),
    .PATTERN             (48'h000000000000),
    .SEL_MASK           ("MASK"),
    .SEL_PATTERN        ("PATTERN"),
    .USE_PATTERN_DETECT ("NO_PATDET"),
    .ACASCREG           (0),
    .ADREG              (0),
    .ALUMODEREG         (0),
    .AREG               (0),
    .BCASCREG           (0),
    .BREG               (0),
    .CARRYINREG         (0),
    .CARRYINSELREG      (0),
    .CREG               (0),
    .DREG               (0),
    .INMODEREG          (0),
    .MREG               (0),
    .OPMODEREG          (0),
    .PREG               (1)
) integrator_2_dsp (
    .CLK                (clk),
    .A                  (30'd0),
    .B                  (18'd0),
    .C                  (48'd0),
    .D                  (25'd0),
    .PCIN               (pcout_1),
    .CARRYIN            (1'b0),
    .CARRYINSEL         (3'b000),
    .OPMODE             (7'b0010010),  // P = P + PCIN
    .ALUMODE            (4'b0000),
    .INMODE             (5'b00000),
    .CEA1               (1'b0),
    .CEA2               (1'b0),
    .CEB1               (1'b0),
    .CEB2               (1'b0),
    .CEC                (1'b0),
    .CED                (1'b0),
    .CEM                (1'b0),
    .CEP                (data_valid),
    .CEAD               (1'b0),
    .CEALUMODE          (1'b0),
    .CECTRL             (1'b0),
    .CECARRYIN          (1'b0),
    .CEINMODE           (1'b0),
    .RSTP               (reset_h),
    .RSTA               (1'b0),
    .RSTB               (1'b0),
    .RSTC               (1'b0),
    .RSTD               (1'b0),
    .RSTM               (1'b0),
    .RSTALLCARRYIN      (1'b0),
    .RSTALUMODE         (1'b0),
    .RSTCTRL            (1'b0),
    .RSTINMODE          (1'b0),
    .P                  (p_out_2),
    .PCOUT              (pcout_2),
    .ACOUT              (),
    .BCOUT              (),
    .CARRYCASCOUT       (),
    .CARRYOUT           (),
    .MULTSIGNOUT        (),
    .OVERFLOW           (),
    .PATTERNBDETECT     (),
    .PATTERNDETECT      (),
    .UNDERFLOW          ()
);

// --- Integrator 3: P = P + PCIN (cascade from integrator_2) ---
DSP48E1 #(
    .A_INPUT            ("DIRECT"),
    .B_INPUT            ("DIRECT"),
    .USE_DPORT          ("FALSE"),
    .USE_MULT           ("NONE"),
    .AUTORESET_PATDET   ("NO_RESET"),
    .MASK               (48'h3FFFFFFFFFFF),
    .PATTERN             (48'h000000000000),
    .SEL_MASK           ("MASK"),
    .SEL_PATTERN        ("PATTERN"),
    .USE_PATTERN_DETECT ("NO_PATDET"),
    .ACASCREG           (0),
    .ADREG              (0),
    .ALUMODEREG         (0),
    .AREG               (0),
    .BCASCREG           (0),
    .BREG               (0),
    .CARRYINREG         (0),
    .CARRYINSELREG      (0),
    .CREG               (0),
    .DREG               (0),
    .INMODEREG          (0),
    .MREG               (0),
    .OPMODEREG          (0),
    .PREG               (1)
) integrator_3_dsp (
    .CLK                (clk),
    .A                  (30'd0),
    .B                  (18'd0),
    .C                  (48'd0),
    .D                  (25'd0),
    .PCIN               (pcout_2),
    .CARRYIN            (1'b0),
    .CARRYINSEL         (3'b000),
    .OPMODE             (7'b0010010),  // P = P + PCIN
    .ALUMODE            (4'b0000),
    .INMODE             (5'b00000),
    .CEA1               (1'b0),
    .CEA2               (1'b0),
    .CEB1               (1'b0),
    .CEB2               (1'b0),
    .CEC                (1'b0),
    .CED                (1'b0),
    .CEM                (1'b0),
    .CEP                (data_valid),
    .CEAD               (1'b0),
    .CEALUMODE          (1'b0),
    .CECTRL             (1'b0),
    .CECARRYIN          (1'b0),
    .CEINMODE           (1'b0),
    .RSTP               (reset_h),
    .RSTA               (1'b0),
    .RSTB               (1'b0),
    .RSTC               (1'b0),
    .RSTD               (1'b0),
    .RSTM               (1'b0),
    .RSTALLCARRYIN      (1'b0),
    .RSTALUMODE         (1'b0),
    .RSTCTRL            (1'b0),
    .RSTINMODE          (1'b0),
    .P                  (p_out_3),
    .PCOUT              (pcout_3),
    .ACOUT              (),
    .BCOUT              (),
    .CARRYCASCOUT       (),
    .CARRYOUT           (),
    .MULTSIGNOUT        (),
    .OVERFLOW           (),
    .PATTERNBDETECT     (),
    .PATTERNDETECT      (),
    .UNDERFLOW          ()
);

// --- Integrator 4: P = P + PCIN (cascade from integrator_3) ---
// No PCOUT needed (last stage in cascade)
DSP48E1 #(
    .A_INPUT            ("DIRECT"),
    .B_INPUT            ("DIRECT"),
    .USE_DPORT          ("FALSE"),
    .USE_MULT           ("NONE"),
    .AUTORESET_PATDET   ("NO_RESET"),
    .MASK               (48'h3FFFFFFFFFFF),
    .PATTERN             (48'h000000000000),
    .SEL_MASK           ("MASK"),
    .SEL_PATTERN        ("PATTERN"),
    .USE_PATTERN_DETECT ("NO_PATDET"),
    .ACASCREG           (0),
    .ADREG              (0),
    .ALUMODEREG         (0),
    .AREG               (0),
    .BCASCREG           (0),
    .BREG               (0),
    .CARRYINREG         (0),
    .CARRYINSELREG      (0),
    .CREG               (0),
    .DREG               (0),
    .INMODEREG          (0),
    .MREG               (0),
    .OPMODEREG          (0),
    .PREG               (1)
) integrator_4_dsp (
    .CLK                (clk),
    .A                  (30'd0),
    .B                  (18'd0),
    .C                  (48'd0),
    .D                  (25'd0),
    .PCIN               (pcout_3),
    .CARRYIN            (1'b0),
    .CARRYINSEL         (3'b000),
    .OPMODE             (7'b0010010),  // P = P + PCIN
    .ALUMODE            (4'b0000),
    .INMODE             (5'b00000),
    .CEA1               (1'b0),
    .CEA2               (1'b0),
    .CEB1               (1'b0),
    .CEB2               (1'b0),
    .CEC                (1'b0),
    .CED                (1'b0),
    .CEM                (1'b0),
    .CEP                (data_valid),
    .CEAD               (1'b0),
    .CEALUMODE          (1'b0),
    .CECTRL             (1'b0),
    .CECARRYIN          (1'b0),
    .CEINMODE           (1'b0),
    .RSTP               (reset_h),
    .RSTA               (1'b0),
    .RSTB               (1'b0),
    .RSTC               (1'b0),
    .RSTD               (1'b0),
    .RSTM               (1'b0),
    .RSTALLCARRYIN      (1'b0),
    .RSTALUMODE         (1'b0),
    .RSTCTRL            (1'b0),
    .RSTINMODE          (1'b0),
    .P                  (p_out_4),
    .PCOUT              (),
    .ACOUT              (),
    .BCOUT              (),
    .CARRYCASCOUT       (),
    .CARRYOUT           (),
    .MULTSIGNOUT        (),
    .OVERFLOW           (),
    .PATTERNBDETECT     (),
    .PATTERNDETECT      (),
    .UNDERFLOW          ()
);

// ============================================================================
// COMB STAGE 0 — Explicit DSP48E1 with CREG=1 for Critical Path Fix
// ============================================================================
// Build 18 critical path: integrator_sampled_comb_reg → comb_reg[0]/C[38]
//   WNS = +0.062 ns, data path = 1.022 ns (0.379 logic + 0.643 route)
//
// By enabling CREG=1 (+ AREG=1, BREG=1), the fabric register
// integrator_sampled_comb is absorbed into the DSP48's internal C pipeline
// register, eliminating the 0.643 ns fabric→DSP routing delay entirely.
// The DSP48 performs: P = C_reg - {A_reg, B_reg}  (i.e., subtract)
//
// Latency: +1 cycle vs. the old inferred comb[0]. This is accounted for
// by the data_valid_comb_0_out signal, which delays the valid for stages 1-4.
//
// C-port = sign-extended integrator_sampled_comb (28→48 bits)
// A:B    = sign-extended comb_delay[0][0] (28→48 bits)
// OPMODE = 7'b0110011: Z=C(011), Y=0(00), X=A:B(11)
// ALUMODE= 4'b0011:    Z - (X + Y + CIN) = C - A:B
//
// The comb_delay[0][0] register stays in fabric (captures
// integrator_sampled_comb at the same time as the C register, unchanged).
// Comb stages 1-4 remain inferred with (* use_dsp = "yes" *).

// Sign-extended inputs for comb_0 DSP48E1
wire [47:0] comb_0_c_in = {{(48-COMB_WIDTH){integrator_sampled_comb[COMB_WIDTH-1]}},
                            integrator_sampled_comb};
wire [47:0] comb_0_ab_in = {{(48-COMB_WIDTH){comb_delay[0][COMB_DELAY-1][COMB_WIDTH-1]}},
                             comb_delay[0][COMB_DELAY-1]};

DSP48E1 #(
    .A_INPUT            ("DIRECT"),
    .B_INPUT            ("DIRECT"),
    .USE_DPORT          ("FALSE"),
    .USE_MULT           ("NONE"),
    .AUTORESET_PATDET   ("NO_RESET"),
    .MASK               (48'h3FFFFFFFFFFF),
    .PATTERN             (48'h000000000000),
    .SEL_MASK           ("MASK"),
    .SEL_PATTERN        ("PATTERN"),
    .USE_PATTERN_DETECT ("NO_PATDET"),
    .ACASCREG           (1),       // A cascade register matches AREG
    .ADREG              (0),
    .ALUMODEREG         (0),
    .AREG               (1),       // A-port registered — eliminates fabric routing
    .BCASCREG           (1),       // B cascade register matches BREG
    .BREG               (1),       // B-port registered — eliminates fabric routing
    .CARRYINREG         (0),
    .CARRYINSELREG      (0),
    .CREG               (1),       // *** KEY: C-port registered inside DSP48 ***
                                   // Absorbs integrator_sampled_comb FDRE, eliminates
                                   // 0.643 ns fabric→DSP C-port routing delay.
    .DREG               (0),
    .INMODEREG          (0),
    .MREG               (0),
    .OPMODEREG          (0),
    .PREG               (1)        // P register enabled (output pipeline)
) comb_0_dsp (
    .CLK                (clk),
    // A:B = sign-extended comb_delay[0][last] (subtrahend)
    .A                  (comb_0_ab_in[47:18]),   // Upper 30 bits
    .B                  (comb_0_ab_in[17:0]),    // Lower 18 bits
    .C                  (comb_0_c_in),           // integrator_sampled_comb (minuend)
    .D                  (25'd0),
    .CARRYIN            (1'b0),
    .CARRYINSEL         (3'b000),
    .OPMODE             (7'b0110011),  // Z=C, Y=0, X=A:B → ALU input = C, A:B
    .ALUMODE            (4'b0011),     // Z - (X+Y+CIN) = C - A:B
    .INMODE             (5'b00000),
    .CEA1               (1'b0),
    .CEA2               (data_valid_comb_pipe),  // Load A register when valid
    .CEB1               (1'b0),
    .CEB2               (data_valid_comb_pipe),  // Load B register when valid
    .CEC                (data_valid_comb_pipe),  // Load C register when valid
    .CED                (1'b0),
    .CEM                (1'b0),
    .CEP                (1'b1),        // Always propagate — P updates 1 cycle after
                                       // input registers are loaded
    .CEAD               (1'b0),
    .CEALUMODE          (1'b0),
    .CECTRL             (1'b0),
    .CECARRYIN          (1'b0),
    .CEINMODE           (1'b0),
    .RSTP               (reset_h),
    .RSTA               (reset_h),
    .RSTB               (reset_h),
    .RSTC               (reset_h),
    .RSTD               (1'b0),
    .RSTM               (1'b0),
    .RSTALLCARRYIN      (1'b0),
    .RSTALUMODE         (1'b0),
    .RSTCTRL            (1'b0),
    .RSTINMODE          (1'b0),
    .P                  (comb_0_p_out),
    .PCOUT              (),
    .ACOUT              (),
    .BCOUT              (),
    .CARRYCASCOUT       (),
    .CARRYOUT           (),
    .MULTSIGNOUT        (),
    .OVERFLOW           (),
    .PATTERNBDETECT     (),
    .PATTERNDETECT      (),
    .UNDERFLOW          ()
);

`else
// ============================================================================
// SIMULATION: Behavioral model (Icarus Verilog compatible)
// ============================================================================
// Functionally identical: each integrator is P <= P + input, gated by data_valid.
// integrator_0 adds sign-extended data_in; stages 1-4 add previous stage output.
//
// CREG=1 on integrator_0: The C-port register adds 1 cycle of latency.
// data_in_c_delayed models this: on cycle N with data_valid, the DSP's C register
// captures data_in_c(N), but the ALU uses the PREVIOUS C register value.
// So sim_int_0 accumulates data_in_c_delayed (1 cycle behind data_in_c).
// ============================================================================
reg signed [ACC_WIDTH-1:0] sim_int_0, sim_int_1, sim_int_2, sim_int_3, sim_int_4;
reg signed [ACC_WIDTH-1:0] data_in_c_delayed;  // Models CREG=1 on integrator_0

// Comb_0 DSP48E1 behavioral model (models CREG+AREG+BREG+PREG pipeline)
// In simulation there is no DSP48E1 primitive, so we model the 4-stage pipe:
//   Stage 1 (CREG/AREG/BREG): capture C and A:B inputs (on data_valid_comb_pipe)
//   Stage 2 (PREG): P = C_reg - AB_reg (always, like CEP=1 in synthesis)
reg signed [COMB_WIDTH-1:0] sim_comb_0_c_reg;   // Models CREG
reg signed [COMB_WIDTH-1:0] sim_comb_0_ab_reg;  // Models AREG+BREG (combined)
reg signed [47:0] sim_comb_0_p_reg;              // Models PREG

always @(posedge clk) begin
    if (reset_h) begin
        sim_int_0 <= 0;
        sim_int_1 <= 0;
        sim_int_2 <= 0;
        sim_int_3 <= 0;
        sim_int_4 <= 0;
        data_in_c_delayed <= 0;
        sim_comb_0_c_reg <= 0;
        sim_comb_0_ab_reg <= 0;
        sim_comb_0_p_reg <= 0;
    end else begin
        if (data_valid) begin
            // CREG pipeline: capture current data, use previous
            data_in_c_delayed <= $signed(data_in_c);
            sim_int_0 <= sim_int_0 + data_in_c_delayed;
            sim_int_1 <= sim_int_1 + sim_int_0;
            sim_int_2 <= sim_int_2 + sim_int_1;
            sim_int_3 <= sim_int_3 + sim_int_2;
            sim_int_4 <= sim_int_4 + sim_int_3;
        end
        // Comb_0 DSP48 behavioral model:
        // CREG/AREG/BREG load on data_valid_comb_pipe (like CEC/CEA2/CEB2)
        if (data_valid_comb_pipe) begin
            sim_comb_0_c_reg <= integrator_sampled_comb;
            sim_comb_0_ab_reg <= comb_delay[0][COMB_DELAY-1];
        end
        // PREG always updates (CEP=1): P = C_reg - AB_reg
        sim_comb_0_p_reg <= {{(48-COMB_WIDTH){sim_comb_0_c_reg[COMB_WIDTH-1]}}, sim_comb_0_c_reg}
                          - {{(48-COMB_WIDTH){sim_comb_0_ab_reg[COMB_WIDTH-1]}}, sim_comb_0_ab_reg};
    end
end

assign comb_0_p_out = sim_comb_0_p_reg;

assign p_out_0 = sim_int_0;
assign p_out_1 = sim_int_1;
assign p_out_2 = sim_int_2;
assign p_out_3 = sim_int_3;
assign p_out_4 = sim_int_4;
// pcout wires unused in simulation
assign pcout_0 = sim_int_0;
assign pcout_1 = sim_int_1;
assign pcout_2 = sim_int_2;
assign pcout_3 = sim_int_3;
`endif

// ============================================================================
// CONTROL AND MONITORING (fabric logic)
// ============================================================================
// (Register declarations moved above `ifndef SIMULATION for Icarus Verilog
//  forward-reference compatibility — see "SHARED REGISTER DECLARATIONS".)

// Initialize
initial begin
    for (i = 0; i < STAGES; i = i + 1) begin
        comb[i] = 0;
        for (j = 0; j < COMB_DELAY; j = j + 1) begin
            comb_delay[i][j] = 0;
        end
    end
    integrator_sampled = 0;
    decimation_counter = 0;
    data_valid_delayed = 0;
    data_valid_comb = 0;
    data_valid_comb_pipe = 0;
    data_valid_comb_0_out = 0;
    output_counter = 0;
    max_integrator_value = 0;
    overflow_detected = 0;
    overflow_latched = 0;
    saturation_detected = 0;
    saturation_event_count = 0;
    sample_count = 0;
    max_value_monitor = 0;
    data_out = 0;
    data_out_valid = 0;
    abs_integrator_value = 0;
    temp_scaled_output = 0;
    temp_output = 0;
    sat_pos = 0;
    sat_neg = 0;
    temp_output_pipe = 0;
    data_out_valid_pipe = 0;
    comb_overflow_latched = 0;
    comb_saturation_detected = 0;
    comb_saturation_event_count = 0;
end

// Decimation control + monitoring (integrators are now DSP48E1 instances)
// Sync reset via reset_h (registered, max_fanout=50) — eliminates the shared
// LUT1 inverter that previously fanned out to all fabric FDRE R pins plus
// DSP48E1 RST pins (702 loads total). See "RESET FAN-OUT INVARIANT" at top.
always @(posedge clk) begin
    if (reset_h) begin
        integrator_sampled <= 0;
        decimation_counter <= 0;
        data_valid_delayed <= 0;
        max_integrator_value <= 0;
        overflow_detected <= 0;
        sample_count <= 0;
        abs_integrator_value <= 0;
        overflow_latched <= 0;
        saturation_detected <= 0;
        saturation_event_count <= 0;
        max_value_monitor <= 0;
        output_counter <= 0;
    end else begin
        // Monitor control
        if (reset_monitors) begin
            overflow_latched <= 0;
            saturation_detected <= 0;
            max_integrator_value <= 0;
            max_value_monitor <= 0;
            saturation_event_count <= 0;
        end

        if (data_valid) begin
            sample_count <= sample_count + 1;

            // Monitor integrator_0 magnitude (read DSP P output)
            abs_integrator_value <= (p_out_0[ACC_WIDTH-1]) ? -$signed(p_out_0) : $signed(p_out_0);

            if (abs_integrator_value > max_integrator_value) begin
                max_integrator_value <= abs_integrator_value;
                max_value_monitor <= abs_integrator_value[27:20];
            end

            // Decimation control
            if (decimation_counter == DECIMATION - 1) begin
                decimation_counter <= 0;
                data_valid_delayed <= 1;
                output_counter <= output_counter + 1;
                // Capture integrator_4 output, truncate to comb width
                integrator_sampled <= p_out_4[COMB_WIDTH-1:0];
            end else begin
                decimation_counter <= decimation_counter + 1;
                data_valid_delayed <= 0;
            end
        end else begin
            data_valid_delayed <= 0;
            overflow_detected <= 1'b0;
        end
    end
end

// Pipeline the valid signal for comb section
// Sync reset via reset_h — same replicated-register source as DSP48E1 RSTs.
always @(posedge clk) begin
    if (reset_h) begin
        data_valid_comb <= 0;
        data_valid_comb_pipe <= 0;
        data_valid_comb_0_out <= 0;
        integrator_sampled_comb <= 0;
    end else begin
        data_valid_comb <= data_valid_delayed;
        data_valid_comb_pipe <= data_valid_comb;
        // data_valid_comb_0_out is delayed 1 cycle from data_valid_comb_pipe
        // to account for CREG+AREG+BREG pipeline in comb_0_dsp.
        // When data_valid_comb_0_out fires, comb_0_p_out (DSP48 PREG)
        // contains the valid comb[0] result.
        data_valid_comb_0_out <= data_valid_comb_pipe;
        integrator_sampled_comb <= integrator_sampled;
    end
end

// Enhanced comb section with scaling and saturation monitoring
// Sync reset: converts FDCE → FDRE for all comb registers. This eliminates
// async-clear routing overhead and enables DSP48E1 absorption of the 28-bit
// subtracts via Vivado's use_dsp inference. The comb subtraction was the
// design-wide critical path in Build 9 (8 logic levels of CARRY4 at 400 MHz,
// WNS = +0.128ns). DSP48E1 ALU performs 48-bit add/subtract in a single
// cycle with zero fabric logic, targeting WNS > +1.0ns.
//
// COMB STAGE 0: Explicit DSP48E1 with CREG=1 (comb_0_dsp instance above).
//   - comb[0] is driven by comb_0_p_out[COMB_WIDTH-1:0] (DSP48 P register)
//   - comb_delay[0][0] still captures integrator_sampled_comb in fabric
//   - Valid signal for stages 1-4 is data_valid_comb_0_out (delayed by 1 cycle
//     from data_valid_comb_pipe to match CREG+AREG+BREG pipeline latency)
//
// COMB STAGES 1-4: Inferred DSP48E1 via (* use_dsp = "yes" *) attribute.
//   - Each stage: comb[i] = comb[i-1] - comb_delay[i][last]

always @(posedge clk) begin
    if (reset_h) begin
        for (i = 0; i < STAGES; i = i + 1) begin
            comb[i] <= 0;
            for (j = 0; j < COMB_DELAY; j = j + 1) begin
                comb_delay[i][j] <= 0;
            end
        end
        data_out <= 0;
        data_out_valid <= 0;
        temp_scaled_output <= 0;
        temp_output <= 0;
        sat_pos <= 0;
        sat_neg <= 0;
        temp_output_pipe <= 0;
        data_out_valid_pipe <= 0;
        comb_overflow_latched <= 0;
        comb_saturation_detected <= 0;
        comb_saturation_event_count <= 0;
    end else begin
        if (reset_monitors) begin
            comb_overflow_latched <= 0;
            comb_saturation_detected <= 0;
            comb_saturation_event_count <= 0;
        end

        // ---- Comb Stage 0: delay line update + DSP48 output capture ----
        // comb_delay[0][0] captures integrator_sampled_comb on the SAME cycle
        // as the DSP48 input registers (CREG/AREG/BREG), so they see the
        // same value. The DSP48 PREG output appears 1 cycle later.
        if (data_valid_comb_pipe) begin
            for (j = COMB_DELAY-1; j > 0; j = j - 1) begin
                comb_delay[0][j] <= comb_delay[0][j-1];
            end
            comb_delay[0][0] <= integrator_sampled_comb;
        end

        // ---- Comb Stage 0 result: from explicit DSP48E1 ----
        // comb_0_dsp PREG output is valid 1 cycle after data_valid_comb_pipe.
        // We capture it into comb[0] on data_valid_comb_0_out so comb stages
        // 1-4 can read it.
        if (data_valid_comb_0_out) begin
            comb[0] <= comb_0_p_out[COMB_WIDTH-1:0];
        end

        // ---- Comb Stages 1-4: inferred DSP48E1 subtracts ----
        // These fire on data_valid_comb_0_out (when comb[0] is valid).
        if (data_valid_comb_0_out) begin
            for (i = 1; i < STAGES; i = i + 1) begin
                comb[i] <= comb[i-1] - comb_delay[i][COMB_DELAY-1];
                for (j = COMB_DELAY-1; j > 0; j = j - 1) begin
                    comb_delay[i][j] <= comb_delay[i][j-1];
                end
                comb_delay[i][0] <= comb[i-1];
            end

            // Gain = (4^5) = 1024 = 2^10, scale by 2^10 to normalize
            temp_scaled_output <= comb[STAGES-1] >>> 10;
            temp_output <= temp_scaled_output[17:0];

            // Pipeline Stage 2: Register saturation comparison flags
            sat_pos <= (temp_scaled_output > 131071);
            sat_neg <= (temp_scaled_output < -131072);
            temp_output_pipe <= temp_scaled_output[17:0];
            data_out_valid_pipe <= 1;
        end else begin
            data_out_valid_pipe <= 0;
        end

        // Pipeline Stage 3: MUX from registered comparison flags
        if (data_out_valid_pipe) begin
            if (sat_pos) begin
                data_out <= 131071;
                comb_overflow_latched <= 1'b1;
                comb_saturation_detected <= 1'b1;
                comb_saturation_event_count <= comb_saturation_event_count + 1;
                `ifdef SIMULATION
                $display("CIC_OUTPUT_SAT: TRUE Positive saturation, final_out=%d", 131071);
                `endif
            end else if (sat_neg) begin
                data_out <= -131072;
                comb_overflow_latched <= 1'b1;
                comb_saturation_detected <= 1'b1;
                comb_saturation_event_count <= comb_saturation_event_count + 1;
                `ifdef SIMULATION
                $display("CIC_OUTPUT_SAT: TRUE Negative saturation, final_out=%d", -131072);
                `endif
            end else begin
                data_out <= temp_output_pipe;
                comb_overflow_latched <= 1'b0;
                comb_saturation_detected <= 1'b0;
            end

            data_out_valid <= 1;
        end else begin
            data_out_valid <= 0;
        end
    end
end

// Continuous monitoring
`ifdef SIMULATION
always @(posedge clk) begin
    if (overflow_detected && sample_count < 100) begin
        $display("CIC_OVERFLOW: Immediate detection at sample %0d", sample_count);
    end
end
`endif

endmodule
