`timescale 1ns / 1ps

// ============================================================================
// Formal Verification Wrapper: radar_mode_controller
// AERIS-10 Radar FPGA — 7-state beam scan FSM
// Target: SymbiYosys with smtbmc/z3
//
// Single-clock design: clk is an input wire, async2sync handles async reset.
// Each formal step = one clock edge.
//
// Timer parameters reduced to small values to keep BMC tractable.
// ============================================================================
module fv_radar_mode_controller (
    input wire clk
);

    // Reduced parameters for tractable BMC
    localparam LONG_CHIRP_CYCLES       = 5;
    localparam LONG_LISTEN_CYCLES      = 5;
    localparam GUARD_CYCLES            = 5;
    localparam SHORT_CHIRP_CYCLES      = 3;
    localparam SHORT_LISTEN_CYCLES     = 3;
    localparam CHIRPS_PER_ELEVATION    = 3;
    localparam ELEVATIONS_PER_AZIMUTH  = 2;
    localparam AZIMUTHS_PER_SCAN       = 2;

    // Maximum timer value across all phases
    localparam MAX_TIMER = LONG_CHIRP_CYCLES;  // 5 (largest)

    // State encoding (mirrors DUT localparams)
    localparam S_IDLE         = 3'd0;
    localparam S_LONG_CHIRP   = 3'd1;
    localparam S_LONG_LISTEN  = 3'd2;
    localparam S_GUARD        = 3'd3;
    localparam S_SHORT_CHIRP  = 3'd4;
    localparam S_SHORT_LISTEN = 3'd5;
    localparam S_ADVANCE      = 3'd6;

`ifdef FORMAL

    // ================================================================
    // Clock is an input wire — smtbmc drives it automatically.
    // async2sync (in .sby, default) converts async reset to sync.
    // ================================================================

    // ================================================================
    // Past-valid tracker (for guarding $past usage)
    // ================================================================
    reg f_past_valid;
    initial f_past_valid = 1'b0;
    always @(posedge clk) f_past_valid <= 1'b1;

    // ================================================================
    // Reset: asserted (low) for cycle 0, deasserted from cycle 1
    // ================================================================
    reg reset_n;
    initial reset_n = 1'b0;
    always @(posedge clk) reset_n <= 1'b1;

    // ================================================================
    // DUT inputs — solver-driven each cycle
    // ================================================================
    (* anyseq *) wire [1:0] mode;
    (* anyseq *) wire       frame_gate;
    (* anyseq *) wire       stm32_new_chirp;
    (* anyseq *) wire       stm32_new_elevation;
    (* anyseq *) wire       stm32_new_azimuth;
    (* anyseq *) wire       trigger;

    // Runtime config inputs are pinned to the reduced localparams so this
    // wrapper proves one tractable configuration. It does not sweep the full
    // runtime-configurable cfg_* space.
    wire [15:0] cfg_long_chirp_cycles   = LONG_CHIRP_CYCLES;
    wire [15:0] cfg_long_listen_cycles  = LONG_LISTEN_CYCLES;
    wire [15:0] cfg_guard_cycles        = GUARD_CYCLES;
    wire [15:0] cfg_short_chirp_cycles  = SHORT_CHIRP_CYCLES;
    wire [15:0] cfg_short_listen_cycles = SHORT_LISTEN_CYCLES;
    wire [5:0]  cfg_chirps_per_elev     = CHIRPS_PER_ELEVATION;

    // ================================================================
    // DUT outputs
    // ================================================================
    wire        use_long_chirp;
    wire        mc_new_chirp;
    wire        mc_new_elevation;
    wire        mc_new_azimuth;
    wire [5:0]  chirp_count;
    wire [5:0]  elevation_count;
    wire [5:0]  azimuth_count;
    wire        scanning;
    wire        scan_complete;
    wire [2:0]  scan_state;
    wire [17:0] timer;

    // ================================================================
    // DUT instantiation
    // ================================================================
    radar_mode_controller #(
        .CHIRPS_PER_ELEVATION   (CHIRPS_PER_ELEVATION),
        .ELEVATIONS_PER_AZIMUTH (ELEVATIONS_PER_AZIMUTH),
        .AZIMUTHS_PER_SCAN      (AZIMUTHS_PER_SCAN),
        .LONG_CHIRP_CYCLES      (LONG_CHIRP_CYCLES),
        .LONG_LISTEN_CYCLES     (LONG_LISTEN_CYCLES),
        .GUARD_CYCLES           (GUARD_CYCLES),
        .SHORT_CHIRP_CYCLES     (SHORT_CHIRP_CYCLES),
        .SHORT_LISTEN_CYCLES    (SHORT_LISTEN_CYCLES)
    ) dut (
        .clk                (clk),
        .reset_n            (reset_n),
        .mode               (mode),
        .frame_gate         (frame_gate),
        .stm32_new_chirp    (stm32_new_chirp),
        .stm32_new_elevation(stm32_new_elevation),
        .stm32_new_azimuth  (stm32_new_azimuth),
        .trigger            (trigger),
        // Runtime-configurable timing ports, bound here to the reduced wrapper
        // constants for tractable proof depth.
        .cfg_long_chirp_cycles  (cfg_long_chirp_cycles),
        .cfg_long_listen_cycles (cfg_long_listen_cycles),
        .cfg_guard_cycles       (cfg_guard_cycles),
        .cfg_short_chirp_cycles (cfg_short_chirp_cycles),
        .cfg_short_listen_cycles(cfg_short_listen_cycles),
        .cfg_chirps_per_elev    (cfg_chirps_per_elev),
        .use_long_chirp     (use_long_chirp),
        .mc_new_chirp       (mc_new_chirp),
        .mc_new_elevation   (mc_new_elevation),
        .mc_new_azimuth     (mc_new_azimuth),
        .chirp_count        (chirp_count),
        .elevation_count    (elevation_count),
        .azimuth_count      (azimuth_count),
        .scanning           (scanning),
        .scan_complete      (scan_complete),
        .fv_scan_state      (scan_state),
        .fv_timer           (timer)
    );

    // scan_state and timer are now accessed via formal output ports

    // ================================================================
    // PROPERTY 1: State encoding — state 7 is unreachable
    // ================================================================
    always @(posedge clk) begin
        if (reset_n)
            assert(scan_state <= 3'd6);
    end

    // ================================================================
    // PROPERTY 2: Counter bounds
    // ================================================================
    always @(posedge clk) begin
        if (reset_n) begin
            assert(chirp_count < CHIRPS_PER_ELEVATION);
            assert(elevation_count < ELEVATIONS_PER_AZIMUTH);
            assert(azimuth_count < AZIMUTHS_PER_SCAN);
        end
    end

    // ================================================================
    // PROPERTY 3: Timer bound
    // Timer must never reach or exceed the maximum timer parameter.
    // The timer counts from 0 to (PARAM - 1) before resetting.
    // ================================================================
    always @(posedge clk) begin
        if (reset_n)
            assert(timer < MAX_TIMER);
    end

    // ================================================================
    // PROPERTY 4: Mode coherence
    // In S_LONG_CHIRP / S_LONG_LISTEN, use_long_chirp must be 1.
    // In S_SHORT_CHIRP / S_SHORT_LISTEN, use_long_chirp must be 0.
    // ================================================================
    always @(posedge clk) begin
        if (reset_n) begin
            if (scan_state == S_LONG_CHIRP || scan_state == S_LONG_LISTEN)
                assert(use_long_chirp == 1'b1);
            if (scan_state == S_SHORT_CHIRP || scan_state == S_SHORT_LISTEN)
                assert(use_long_chirp == 1'b0);
        end
    end

    // ================================================================
    // PROPERTY 5: Single-chirp returns to idle
    // In mode 2'b10, after S_LONG_LISTEN completes (timer reaches
    // max), scan_state returns to S_IDLE.
    // ================================================================
    always @(posedge clk) begin
        if (reset_n && f_past_valid) begin
            if ($past(mode) == 2'b10 && mode == 2'b10 &&
                $past(scan_state) == S_LONG_LISTEN &&
                $past(timer) == LONG_LISTEN_CYCLES - 1)
                assert(scan_state == S_IDLE);
        end
    end

    // ================================================================
    // PROPERTY 6: Gated auto-scan (mode 01) starts on frame_gate
    // In mode 2'b01, the FSM must leave S_IDLE on the cycle after a
    // frame_gate pulse, and must stay in S_IDLE while no gate arrives.
    // ================================================================
    always @(posedge clk) begin
        if (reset_n && f_past_valid) begin
            // Gate pulse seen in S_IDLE → must start on the next cycle
            if ($past(mode) == 2'b01 && $past(scan_state) == S_IDLE &&
                $past(frame_gate) == 1'b1 && mode == 2'b01)
                assert(scan_state != S_IDLE);
            // No gate in S_IDLE → must not start
            if ($past(mode) == 2'b01 && $past(scan_state) == S_IDLE &&
                $past(frame_gate) == 1'b0 && mode == 2'b01)
                assert(scan_state == S_IDLE);
        end
    end

    // ================================================================
    // COVER 1: Full auto-scan completes
    // ================================================================
    always @(posedge clk) begin
        if (reset_n)
            cover(scan_complete && mode == 2'b01);
    end

    // ================================================================
    // COVER 2: Each state is reachable
    // ================================================================
    always @(posedge clk) begin
        if (reset_n) begin
            cover(scan_state == S_IDLE);
            cover(scan_state == S_LONG_CHIRP);
            cover(scan_state == S_LONG_LISTEN);
            cover(scan_state == S_GUARD);
            cover(scan_state == S_SHORT_CHIRP);
            cover(scan_state == S_SHORT_LISTEN);
            cover(scan_state == S_ADVANCE);
        end
    end

`endif // FORMAL

endmodule
