`timescale 1ns / 1ps

/**
 * rx_gain_control.v
 *
 * Digital gain control with optional per-frame automatic gain control (AGC)
 * for the receive path. Placed between DDC output and matched filter input.
 *
 * Manual mode (agc_enable=0):
 *   - Uses host_gain_shift directly (backward-compatible, no behavioral change)
 *   - gain_shift[3]   = direction: 0 = left shift (amplify), 1 = right shift (attenuate)
 *   - gain_shift[2:0] = amount: 0..7 bits
 *   - Symmetric saturation to ±32767 on overflow
 *
 * AGC mode (agc_enable=1):
 *   - Per-frame automatic gain adjustment based on peak/saturation metrics
 *   - Internal signed gain: -7 (max attenuation) to +7 (max amplification)
 *   - On frame_boundary:
 *       * If saturation detected: gain -= agc_attack (fast, immediate)
 *       * Else if peak < target after holdoff frames: gain += agc_decay (slow)
 *       * Else: hold current gain
 *   - host_gain_shift serves as initial gain when AGC first enabled
 *
 * Status outputs (for readback via status_words):
 *   - current_gain[3:0]: effective gain_shift encoding (manual or AGC)
 *   - peak_magnitude[7:0]: per-frame peak |sample| (upper 8 bits of 15-bit value)
 *   - saturation_count[7:0]: per-frame clipped sample count (capped at 255)
 *
 * Timing: 1-cycle data latency, valid-in/valid-out pipeline.
 *
 * Insertion point in radar_receiver_final.v:
 *   ddc_input_interface → rx_gain_control → matched_filter_multi_segment
 */

module rx_gain_control (
    input  wire        clk,
    input  wire        reset_n,

    // Data input (from DDC / ddc_input_interface)
    input  wire signed [15:0] data_i_in,
    input  wire signed [15:0] data_q_in,
    input  wire               valid_in,

    // Host gain configuration (from USB command opcode 0x16)
    // [3]=direction: 0=amplify (left shift), 1=attenuate (right shift)
    // [2:0]=shift amount: 0..7 bits. Default 0x00 = pass-through.
    // In AGC mode: serves as initial gain on AGC enable transition.
    input  wire [3:0]  gain_shift,

    // AGC configuration inputs (from host via USB, opcodes 0x28-0x2C)
    input  wire        agc_enable,     // 0x28: 0=manual gain, 1=auto AGC
    input  wire [7:0]  agc_target,     // 0x29: target peak magnitude (unsigned, default 200)
    input  wire [3:0]  agc_attack,     // 0x2A: attenuation step on clipping (default 1)
    input  wire [3:0]  agc_decay,      // 0x2B: amplification step when weak (default 1)
    input  wire [3:0]  agc_holdoff,    // 0x2C: frames to wait before gain-up (default 4)

    // Frame boundary pulse (1 clk cycle, from Doppler frame_complete)
    input  wire        frame_boundary,

    // Data output (to matched filter)
    output reg  signed [15:0] data_i_out,
    output reg  signed [15:0] data_q_out,
    output reg                valid_out,

    // Diagnostics / status readback
    output reg  [7:0]  saturation_count,  // Per-frame clipped sample count (capped at 255)
    output reg  [7:0]  peak_magnitude,    // Per-frame peak |sample| (upper 8 bits of 15-bit)
    output reg  [3:0]  current_gain       // Current effective gain_shift (for status readback)
);

// =========================================================================
// INTERNAL AGC STATE
// =========================================================================

// Signed internal gain: -7 (max attenuation) to +7 (max amplification)
// Stored as 4-bit signed (range -8..+7, clamped to -7..+7)
reg signed [3:0] agc_gain;

// Holdoff counter: counts frames without saturation before allowing gain-up
reg [3:0] holdoff_counter;

// Per-frame accumulators (running, reset on frame_boundary)
reg [7:0]  frame_sat_count;    // Clipped samples this frame
reg [14:0] frame_peak;         // Peak |sample| this frame (15-bit unsigned)

// Previous AGC enable state (for detecting 0→1 transition)
reg agc_enable_prev;

// Combinational helpers for inclusive frame-boundary snapshot
// (used when valid_in and frame_boundary coincide)
reg wire_frame_sat_incr;
reg wire_frame_peak_update;

// =========================================================================
// EFFECTIVE GAIN SELECTION
// =========================================================================

// Convert between signed internal gain and the gain_shift[3:0] encoding.
//   gain_shift[3]=0, [2:0]=N → amplify by N bits (internal gain = +N)
//   gain_shift[3]=1, [2:0]=N → attenuate by N bits (internal gain = -N)

// Effective gain_shift used for the actual shift operation
wire [3:0] effective_gain;
assign effective_gain = agc_enable ? current_gain : gain_shift;

// Decompose effective gain for shift logic
wire       shift_right = effective_gain[3];
wire [2:0] shift_amt   = effective_gain[2:0];

// =========================================================================
// COMBINATIONAL SHIFT + SATURATION
// =========================================================================
// Use wider intermediates to detect overflow on left shift.
// 24 bits is enough: 16 + 7 shift = 23 significant bits max.

wire signed [23:0] shifted_i;
wire signed [23:0] shifted_q;

assign shifted_i = shift_right ? (data_i_in >>> shift_amt)
                               : (data_i_in <<< shift_amt);
assign shifted_q = shift_right ? (data_q_in >>> shift_amt)
                               : (data_q_in <<< shift_amt);

// Saturation: clamp to signed 16-bit range [-32768, +32767]
wire overflow_i = (shifted_i > 24'sd32767) || (shifted_i < -24'sd32768);
wire overflow_q = (shifted_q > 24'sd32767) || (shifted_q < -24'sd32768);

wire signed [15:0] sat_i = overflow_i ? (shifted_i[23] ? -16'sd32768 : 16'sd32767)
                                      : shifted_i[15:0];
wire signed [15:0] sat_q = overflow_q ? (shifted_q[23] ? -16'sd32768 : 16'sd32767)
                                      : shifted_q[15:0];

// =========================================================================
// PEAK MAGNITUDE TRACKING (combinational)
// =========================================================================
// Absolute value of signed 16-bit: flip sign bit if negative.
// Result is 15-bit unsigned [0, 32767]. (We ignore -32768 → 32767 edge case.)
wire [14:0] abs_i = data_i_in[15] ? (~data_i_in[14:0] + 15'd1) : data_i_in[14:0];
wire [14:0] abs_q = data_q_in[15] ? (~data_q_in[14:0] + 15'd1) : data_q_in[14:0];
wire [14:0] max_iq = (abs_i > abs_q) ? abs_i : abs_q;

// =========================================================================
// SIGNED GAIN ↔ GAIN_SHIFT ENCODING CONVERSION
// =========================================================================
// Convert signed agc_gain to gain_shift[3:0] encoding
function [3:0] signed_to_encoding;
    input signed [3:0] g;
    begin
        if (g >= 0)
            signed_to_encoding = {1'b0, g[2:0]};       // amplify
        else
            signed_to_encoding = {1'b1, (~g[2:0]) + 3'd1}; // attenuate: -g
    end
endfunction

// Convert gain_shift[3:0] encoding to signed gain
function signed [3:0] encoding_to_signed;
    input [3:0] enc;
    begin
        if (enc[3] == 1'b0)
            encoding_to_signed = {1'b0, enc[2:0]};     // +0..+7
        else
            encoding_to_signed = -$signed({1'b0, enc[2:0]}); // -1..-7
    end
endfunction

// =========================================================================
// CLAMPING HELPER
// =========================================================================
// Clamp a wider signed value to [-7, +7]
function signed [3:0] clamp_gain;
    input signed [5:0] val;  // 6-bit: covers [-22,+22] (max |gain|+step = 7+15)
    begin
        if (val > 6'sd7)
            clamp_gain = 4'sd7;
        else if (val < -6'sd7)
            clamp_gain = -4'sd7;
        else
            clamp_gain = val[3:0];
    end
endfunction

// =========================================================================
// REGISTERED OUTPUT + AGC STATE MACHINE
// =========================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        // Data path
        data_i_out       <= 16'sd0;
        data_q_out       <= 16'sd0;
        valid_out        <= 1'b0;
        // Status outputs
        saturation_count <= 8'd0;
        peak_magnitude   <= 8'd0;
        current_gain     <= 4'd0;
        // AGC internal state
        agc_gain         <= 4'sd0;
        holdoff_counter  <= 4'd0;
        frame_sat_count  <= 8'd0;
        frame_peak       <= 15'd0;
        agc_enable_prev  <= 1'b0;
    end else begin
        // Track AGC enable transitions
        agc_enable_prev <= agc_enable;

        // Compute inclusive metrics: if valid_in fires this cycle,
        // include current sample in the snapshot taken at frame_boundary.
        // This avoids losing the last sample when valid_in and
        // frame_boundary coincide (NBA last-write-wins would otherwise
        // snapshot stale values then reset, dropping the sample entirely).
        wire_frame_sat_incr = (valid_in && (overflow_i || overflow_q)
                               && (frame_sat_count != 8'hFF));
        wire_frame_peak_update = (valid_in && (max_iq > frame_peak));

        // ---- Data pipeline (1-cycle latency) ----
        valid_out <= valid_in;
        if (valid_in) begin
            data_i_out <= sat_i;
            data_q_out <= sat_q;

            // Per-frame saturation counting
            if ((overflow_i || overflow_q) && (frame_sat_count != 8'hFF))
                frame_sat_count <= frame_sat_count + 8'd1;

            // Per-frame peak tracking (pre-gain, measures input signal level)
            if (max_iq > frame_peak)
                frame_peak <= max_iq;
        end

        // ---- Frame boundary: AGC update + metric snapshot ----
        if (frame_boundary) begin
            // Snapshot per-frame metrics INCLUDING current sample if valid_in
            saturation_count <= wire_frame_sat_incr
                                ? (frame_sat_count + 8'd1)
                                : frame_sat_count;
            peak_magnitude   <= wire_frame_peak_update
                                ? max_iq[14:7]
                                : frame_peak[14:7];

            // Reset per-frame accumulators for next frame
            frame_sat_count <= 8'd0;
            frame_peak      <= 15'd0;

            if (agc_enable) begin
                // AGC auto-adjustment at frame boundary
                // Use inclusive counts/peaks (accounting for simultaneous valid_in)
                if (wire_frame_sat_incr || frame_sat_count > 8'd0) begin
                    // Clipping detected: reduce gain immediately (attack)
                    agc_gain <= clamp_gain($signed({agc_gain[3], agc_gain[3], agc_gain}) -
                                           $signed({2'b00, agc_attack}));
                    holdoff_counter <= agc_holdoff;  // Reset holdoff
                end else if ((wire_frame_peak_update ? max_iq[14:7] : frame_peak[14:7])
                             < agc_target) begin
                    // Signal too weak: increase gain after holdoff expires
                    if (holdoff_counter == 4'd0) begin
                        agc_gain <= clamp_gain($signed({agc_gain[3], agc_gain[3], agc_gain}) +
                                               $signed({2'b00, agc_decay}));
                    end else begin
                        holdoff_counter <= holdoff_counter - 4'd1;
                    end
                end else begin
                    // Signal in good range, no saturation: hold gain
                    // Reset holdoff so next weak frame has to wait again
                    holdoff_counter <= agc_holdoff;
                end
            end
        end

        // ---- AGC enable transition: initialize from host gain ----
        if (agc_enable && !agc_enable_prev) begin
            agc_gain        <= encoding_to_signed(gain_shift);
            holdoff_counter <= agc_holdoff;
        end

        // ---- Update current_gain output ----
        if (agc_enable)
            current_gain <= signed_to_encoding(agc_gain);
        else
            current_gain <= gain_shift;
    end
end

endmodule
