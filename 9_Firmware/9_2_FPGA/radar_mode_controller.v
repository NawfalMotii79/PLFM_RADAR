`timescale 1ns / 1ps

/**
 * radar_mode_controller.v
 *
 * Generates beam scanning and chirp mode control signals for the AERIS-10
 * receiver processing chain. This module drives:
 *   - use_long_chirp   : selects long (30us) or short (0.5us) chirp mode
 *   - mc_new_chirp     : toggle signal indicating new chirp start
 *   - mc_new_elevation : toggle signal indicating elevation step
 *   - mc_new_azimuth   : toggle signal indicating azimuth step
 *
 * These signals are consumed by matched_filter_multi_segment and
 * chirp_memory_loader_param in the receiver path.
 *
 * The controller mirrors the transmitter's chirp sequence defined in
 * plfm_chirp_controller_enhanced:
 *   - 32 chirps per elevation
 *   - 31 elevations per azimuth
 *   - 50 azimuths per full scan
 *   - Each chirp: Long chirp → Listen → Guard → Short chirp → Listen
 *
 * Modes of operation:
 *   mode[1:0]:
 *     2'b00 = STM32-driven (pass through stm32 toggle signals)
 *     2'b01 = Gated auto-scan (internal timing, frame started by frame_gate;
 *             RX frames lock 1:1 to the MCU-triggered TX bursts)
 *     2'b10 = Single-chirp (fire one chirp per trigger, for debug)
 *     2'b11 = Reserved
 *
 * Clock domain: clk (100 MHz)
 */

module radar_mode_controller #(
    parameter CHIRPS_PER_ELEVATION = 32,
    parameter ELEVATIONS_PER_AZIMUTH = 31,
    parameter AZIMUTHS_PER_SCAN = 50,

    // Timing in 100 MHz clock cycles
    // Long chirp: 30us = 3000 cycles at 100 MHz
    // Long listen: 137us = 13700 cycles
    // Guard: 175.4us = 17540 cycles
    // Short chirp: 0.5us = 50 cycles
    // Short listen: 174.5us = 17450 cycles
    parameter LONG_CHIRP_CYCLES   = 3000,
    parameter LONG_LISTEN_CYCLES  = 13700,
    parameter GUARD_CYCLES        = 17540,
    parameter SHORT_CHIRP_CYCLES  = 50,
    parameter SHORT_LISTEN_CYCLES = 17450
) (
    input wire clk,
    input wire reset_n,

    // Mode selection
    input wire [1:0] mode,          // 00=STM32, 01=auto, 10=single, 11=rsvd

    // Frame gate (mode 01): 1-cycle pulse that starts a receiver frame.
    // Driven by the transmitter's new_chirp_frame so the RX frame starts
    // exactly when the MCU-triggered TX burst starts (hardware-aligned chirp
    // windows). In production the MCU fires one burst per beam block, so the
    // RX must not free-run between bursts — it waits in S_IDLE for the gate.
    input wire frame_gate,

    // STM32 pass-through inputs (active in mode 00)
    input wire stm32_new_chirp,
    input wire stm32_new_elevation,
    input wire stm32_new_azimuth,

    // Single-chirp trigger (active in mode 10)
    input wire trigger,

    // Gap 2: Runtime-configurable timing inputs from host USB commands.
    // When connected, these override the compile-time parameters.
    // When left at default (tied to parameter values at instantiation),
    // behavior is identical to pre-Gap-2.
    input wire [15:0] cfg_long_chirp_cycles,
    input wire [15:0] cfg_long_listen_cycles,
    input wire [15:0] cfg_guard_cycles,
    input wire [15:0] cfg_short_chirp_cycles,
    input wire [15:0] cfg_short_listen_cycles,
    input wire [5:0]  cfg_chirps_per_elev,

    // Outputs to receiver processing chain
    output reg use_long_chirp,
    output reg mc_new_chirp,
    output reg mc_new_elevation,
    output reg mc_new_azimuth,

    // Beam position tracking
    output reg [5:0] chirp_count,
    output reg [5:0] elevation_count,
    output reg [5:0] azimuth_count,

    // Status
    output wire scanning,       // 1 = scan in progress
    output wire scan_complete   // pulse when full scan done

`ifdef FORMAL
    ,
    output wire [2:0]  fv_scan_state,
    output wire [17:0] fv_timer
`endif
);

// ============================================================================
// INTERNAL STATE
// ============================================================================

// Auto-scan state machine
reg [2:0] scan_state;
localparam S_IDLE        = 3'd0;
localparam S_LONG_CHIRP  = 3'd1;
localparam S_LONG_LISTEN = 3'd2;
localparam S_GUARD       = 3'd3;
localparam S_SHORT_CHIRP = 3'd4;
localparam S_SHORT_LISTEN = 3'd5;
localparam S_ADVANCE     = 3'd6;

// Timing counter
reg [17:0] timer;  // enough for up to 262143 cycles (~2.6ms at 100 MHz)

`ifdef FORMAL
assign fv_scan_state = scan_state;
assign fv_timer      = timer;
`endif

// Edge detection for STM32 pass-through
reg stm32_new_chirp_prev;
reg stm32_new_elevation_prev;
reg stm32_new_azimuth_prev;

// Trigger edge detection (for single-chirp mode)
reg trigger_prev;
wire trigger_pulse = trigger & ~trigger_prev;

// Scan completion
reg scan_done_pulse;

// ============================================================================
// EDGE DETECTION
// ============================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        stm32_new_chirp_prev     <= 1'b0;
        stm32_new_elevation_prev <= 1'b0;
        stm32_new_azimuth_prev   <= 1'b0;
        trigger_prev             <= 1'b0;
    end else begin
        stm32_new_chirp_prev     <= stm32_new_chirp;
        stm32_new_elevation_prev <= stm32_new_elevation;
        stm32_new_azimuth_prev   <= stm32_new_azimuth;
        trigger_prev             <= trigger;
    end
end

wire stm32_chirp_toggle     = stm32_new_chirp     ^ stm32_new_chirp_prev;
wire stm32_elevation_toggle = stm32_new_elevation  ^ stm32_new_elevation_prev;
wire stm32_azimuth_toggle   = stm32_new_azimuth    ^ stm32_new_azimuth_prev;

// ============================================================================
// MAIN STATE MACHINE
// ============================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        scan_state      <= S_IDLE;
        timer           <= 18'd0;
        use_long_chirp  <= 1'b1;
        mc_new_chirp    <= 1'b0;
        mc_new_elevation <= 1'b0;
        mc_new_azimuth  <= 1'b0;
        chirp_count     <= 6'd0;
        elevation_count <= 6'd0;
        azimuth_count   <= 6'd0;
        scan_done_pulse <= 1'b0;
    end else begin
        // Clear one-shot signals
        scan_done_pulse <= 1'b0;

        case (mode)
        // ================================================================
        // MODE 00: STM32-driven pass-through
        // The STM32 firmware controls timing; we just detect toggle edges
        // and forward them to the receiver chain.
        // ================================================================
        2'b00: begin
            // Reset auto-scan state
            scan_state <= S_IDLE;
            timer      <= 18'd0;

            // Pass through toggle signals
            if (stm32_chirp_toggle) begin
                mc_new_chirp <= ~mc_new_chirp;  // Toggle output
                use_long_chirp <= 1'b1;         // Default to long chirp

                // Track chirp count (Gap 2: use runtime cfg_chirps_per_elev)
                if (chirp_count < cfg_chirps_per_elev - 1)
                    chirp_count <= chirp_count + 1;
                else
                    chirp_count <= 6'd0;
            end

            if (stm32_elevation_toggle) begin
                mc_new_elevation <= ~mc_new_elevation;
                chirp_count <= 6'd0;

                if (elevation_count < ELEVATIONS_PER_AZIMUTH - 1)
                    elevation_count <= elevation_count + 1;
                else
                    elevation_count <= 6'd0;
            end

            if (stm32_azimuth_toggle) begin
                mc_new_azimuth <= ~mc_new_azimuth;
                elevation_count <= 6'd0;

                if (azimuth_count < AZIMUTHS_PER_SCAN - 1)
                    azimuth_count <= azimuth_count + 1;
                else begin
                    azimuth_count <= 6'd0;
                    scan_done_pulse <= 1'b1;
                end
            end
        end

        // ================================================================
        // MODE 01: Free-running auto-scan
        // Internally generates chirp timing matching the transmitter.
        // ================================================================
        2'b01: begin
            case (scan_state)
            S_IDLE: begin
                // Gated start: wait for the transmitter burst (frame_gate).
                // Prevents RX drift when the MCU gates TX bursts with gaps
                // (beam-pattern SPI writes between bursts).
                if (frame_gate) begin
                    scan_state     <= S_LONG_CHIRP;
                    timer          <= 18'd0;
                    use_long_chirp <= 1'b1;
                    mc_new_chirp   <= ~mc_new_chirp;  // Toggle to start chirp
                    chirp_count    <= 6'd0;
                    // NOTE: elevation_count/azimuth_count are NOT reset here —
                    // they accumulate across gated frames (full-scan wrap
                    // tracking happens in S_ADVANCE).

                    `ifdef SIMULATION
                    $display("[MODE_CTRL] Auto-scan frame started (gated)");
                    `endif
                end
            end

            S_LONG_CHIRP: begin
                use_long_chirp <= 1'b1;
                if (timer < cfg_long_chirp_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_LONG_LISTEN;
                end
            end

            S_LONG_LISTEN: begin
                if (timer < cfg_long_listen_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_GUARD;
                end
            end

            S_GUARD: begin
                if (timer < cfg_guard_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_SHORT_CHIRP;
                    use_long_chirp <= 1'b0;
                end
            end

            S_SHORT_CHIRP: begin
                use_long_chirp <= 1'b0;
                if (timer < cfg_short_chirp_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_SHORT_LISTEN;
                end
            end

            S_SHORT_LISTEN: begin
                if (timer < cfg_short_listen_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_ADVANCE;
                end
            end

            S_ADVANCE: begin
                // Advance chirp/elevation/azimuth counters
                // (Gap 2: use runtime cfg_chirps_per_elev)
                if (chirp_count < cfg_chirps_per_elev - 1) begin
                    // Next chirp in current elevation
                    chirp_count  <= chirp_count + 1;
                    mc_new_chirp <= ~mc_new_chirp;
                    scan_state   <= S_LONG_CHIRP;
                    use_long_chirp <= 1'b1;
                end else begin
                    // Frame complete = cfg_chirps_per_elev chirps = one TX
                    // burst. Return to S_IDLE and wait for the next
                    // frame_gate so RX stays locked to the MCU-gated bursts.
                    // No mc_new_chirp toggle here — the next frame's first
                    // chirp toggles on the gate in S_IDLE.
                    chirp_count <= 6'd0;

                    if (elevation_count < ELEVATIONS_PER_AZIMUTH - 1) begin
                        // Next elevation (cosmetic scan tracking)
                        elevation_count  <= elevation_count + 1;
                        mc_new_elevation <= ~mc_new_elevation;
                    end else begin
                        elevation_count <= 6'd0;

                        if (azimuth_count < AZIMUTHS_PER_SCAN - 1) begin
                            // Next azimuth (cosmetic scan tracking)
                            azimuth_count    <= azimuth_count + 1;
                            mc_new_elevation <= ~mc_new_elevation;
                            mc_new_azimuth   <= ~mc_new_azimuth;
                        end else begin
                            // Full scan complete — restart next frame
                            azimuth_count   <= 6'd0;
                            scan_done_pulse <= 1'b1;
                            mc_new_elevation <= ~mc_new_elevation;
                            mc_new_azimuth  <= ~mc_new_azimuth;

                            `ifdef SIMULATION
                            $display("[MODE_CTRL] Full scan complete, waiting for gate");
                            `endif
                        end
                    end

                    scan_state     <= S_IDLE;
                    use_long_chirp <= 1'b1;
                end
            end

            default: scan_state <= S_IDLE;
            endcase
        end

        // ================================================================
        // MODE 10: Single-chirp (debug mode)
        // Fire one long chirp per trigger pulse, no scanning.
        // ================================================================
        2'b10: begin
            case (scan_state)
            S_IDLE: begin
                if (trigger_pulse) begin
                    scan_state     <= S_LONG_CHIRP;
                    timer          <= 18'd0;
                    use_long_chirp <= 1'b1;
                    mc_new_chirp   <= ~mc_new_chirp;
                end
            end

            S_LONG_CHIRP: begin
                if (timer < cfg_long_chirp_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_LONG_LISTEN;
                end
            end

            S_LONG_LISTEN: begin
                if (timer < cfg_long_listen_cycles - 1)
                    timer <= timer + 1;
                else begin
                    // Single chirp done, return to idle
                    timer      <= 18'd0;
                    scan_state <= S_IDLE;
                end
            end

            default: scan_state <= S_IDLE;
            endcase
        end

        // ================================================================
        // MODE 11: Reserved — idle
        // ================================================================
        2'b11: begin
            scan_state <= S_IDLE;
            timer      <= 18'd0;
        end

        endcase
    end
end

// ============================================================================
// OUTPUT ASSIGNMENTS
// ============================================================================
assign scanning      = (scan_state != S_IDLE);
assign scan_complete = scan_done_pulse;

endmodule
