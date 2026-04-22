`timescale 1ns / 1ps
`include "radar_params.vh"

module chirp_memory_loader_param #(
    parameter LONG_I_FILE_SEG0  = "long_chirp_seg0_i.mem",
    parameter LONG_Q_FILE_SEG0  = "long_chirp_seg0_q.mem",
    parameter LONG_I_FILE_SEG1  = "long_chirp_seg1_i.mem",
    parameter LONG_Q_FILE_SEG1  = "long_chirp_seg1_q.mem",
    parameter SHORT_I_FILE      = "short_chirp_i.mem",
    parameter SHORT_Q_FILE      = "short_chirp_q.mem",
    parameter MEDIUM_I_FILE     = "medium_chirp_i.mem",
    parameter MEDIUM_Q_FILE     = "medium_chirp_q.mem",
    // MEDIUM_CHIRP_SAMPLES: active samples in medium chirp — sourced from radar_params.vh
    parameter MEDIUM_CHIRP_SAMPLES = `RP_MEDIUM_CHIRP_SAMPLES_RX,  // 500 at 100 MHz
    parameter DEBUG = 1
)(
    input wire        clk,
    input wire        reset_n,
    // cfg_range_mode: RP_RANGE_MODE_3KM=2'b00 (medium chirp only), RP_RANGE_MODE_20KM=2'b01 (long+short)
    input wire [1:0]  cfg_range_mode,
    input wire [1:0]  segment_select,
    input wire        mem_request,
    // use_long_chirp: dynamic toggle from radar_mode_controller (ignored in 3km mode)
    input wire        use_long_chirp,
    input wire [10:0] sample_addr,
    output reg [15:0] ref_i,
    output reg [15:0] ref_q,
    output reg        mem_ready
);

// ---- BRAM declarations ----
// Long chirp: 2 segments × 2048 = 4096 samples (20km mode)
(* ram_style = "block" *) reg [15:0] long_chirp_i  [0:4095];
(* ram_style = "block" *) reg [15:0] long_chirp_q  [0:4095];
// Short chirp: 2048 samples, first 50 active (20km disambiguation)
(* ram_style = "block" *) reg [15:0] short_chirp_i [0:2047];
(* ram_style = "block" *) reg [15:0] short_chirp_q [0:2047];
// Medium chirp: RP_MEDIUM_CHIRP_MEM_DEPTH entries, first MEDIUM_CHIRP_SAMPLES active (3km)
(* ram_style = "block" *) reg [15:0] medium_chirp_i [0:`RP_MEDIUM_CHIRP_MEM_DEPTH-1];
(* ram_style = "block" *) reg [15:0] medium_chirp_q [0:`RP_MEDIUM_CHIRP_MEM_DEPTH-1];

integer i;

initial begin
    `ifdef SIMULATION
    if (DEBUG) $display("[MEM] Initialising chirp memories (2 long segs + short + medium)");
    `endif

    // === LONG CHIRP — 2 × 2048 segments (20km) ===
    $readmemh(LONG_I_FILE_SEG0, long_chirp_i, 0, 2047);
    $readmemh(LONG_Q_FILE_SEG0, long_chirp_q, 0, 2047);
    $readmemh(LONG_I_FILE_SEG1, long_chirp_i, 2048, 4095);
    $readmemh(LONG_Q_FILE_SEG1, long_chirp_q, 2048, 4095);
    `ifdef SIMULATION
    if (DEBUG) $display("[MEM] Loaded long chirp segs 0-1 (0-4095)");
    `endif

    // === SHORT CHIRP — 50 active, zero-pad to 2047 (20km disambiguation) ===
    $readmemh(SHORT_I_FILE, short_chirp_i, 0, 49);
    $readmemh(SHORT_Q_FILE, short_chirp_q, 0, 49);
    for (i = 50; i < 2048; i = i + 1) begin
        short_chirp_i[i] = 16'h0000;
        short_chirp_q[i] = 16'h0000;
    end
    `ifdef SIMULATION
    if (DEBUG) $display("[MEM] Loaded short chirp (0-49), zero-padded 50-2047");
    `endif

    // === MEDIUM CHIRP — MEDIUM_CHIRP_SAMPLES active, zero-pad to 1023 (3km) ===
    $readmemh(MEDIUM_I_FILE, medium_chirp_i, 0, MEDIUM_CHIRP_SAMPLES - 1);
    $readmemh(MEDIUM_Q_FILE, medium_chirp_q, 0, MEDIUM_CHIRP_SAMPLES - 1);
    for (i = MEDIUM_CHIRP_SAMPLES; i < 1024; i = i + 1) begin
        medium_chirp_i[i] = 16'h0000;
        medium_chirp_q[i] = 16'h0000;
    end
    `ifdef SIMULATION
    if (DEBUG) $display("[MEM] Loaded medium chirp (0-%0d), zero-padded to 1023",
                        MEDIUM_CHIRP_SAMPLES - 1);

    // === VERIFICATION ===
    if (DEBUG) begin
        $display("[MEM] Verification samples:");
        $display("  Long[0]:      I=%h Q=%h", long_chirp_i[0],    long_chirp_q[0]);
        $display("  Long[2047]:   I=%h Q=%h", long_chirp_i[2047], long_chirp_q[2047]);
        $display("  Long[2048]:   I=%h Q=%h", long_chirp_i[2048], long_chirp_q[2048]);
        $display("  Long[4095]:   I=%h Q=%h", long_chirp_i[4095], long_chirp_q[4095]);
        $display("  Short[0]:     I=%h Q=%h", short_chirp_i[0],   short_chirp_q[0]);
        $display("  Short[49]:    I=%h Q=%h", short_chirp_i[49],  short_chirp_q[49]);
        $display("  Short[50]:    I=%h Q=%h (zero-padded)", short_chirp_i[50], short_chirp_q[50]);
        $display("  Medium[0]:    I=%h Q=%h", medium_chirp_i[0],  medium_chirp_q[0]);
        $display("  Medium[%0d]: I=%h Q=%h", MEDIUM_CHIRP_SAMPLES - 1,
                 medium_chirp_i[MEDIUM_CHIRP_SAMPLES-1], medium_chirp_q[MEDIUM_CHIRP_SAMPLES-1]);
    end
    `endif
end

// long_addr: segment_select[0] picks segment, sample_addr[10:0] within segment
wire [11:0] long_addr   = {segment_select[0], sample_addr};
// medium_addr: lower 10 bits of sample_addr (medium array is 1024 deep)
wire [9:0]  medium_addr = sample_addr[9:0];

// ---- BRAM read — synchronous reset (REQP-1839/1840) ----
always @(posedge clk) begin
    if (!reset_n) begin
        ref_i <= 16'd0;
        ref_q <= 16'd0;
    end else if (mem_request) begin
        if (cfg_range_mode == `RP_RANGE_MODE_3KM) begin
            // 3km mode: medium chirp only — long/short LUTs unused
            ref_i <= medium_chirp_i[medium_addr];
            ref_q <= medium_chirp_q[medium_addr];

            `ifdef SIMULATION
            if (DEBUG && $time < 100)
                $display("[MEM @%0t] Medium chirp: addr=%d, I=%h, Q=%h",
                         $time, medium_addr,
                         medium_chirp_i[medium_addr], medium_chirp_q[medium_addr]);
            `endif
        end else if (use_long_chirp) begin
            // 20km mode, long-chirp phase
            ref_i <= long_chirp_i[long_addr];
            ref_q <= long_chirp_q[long_addr];

            `ifdef SIMULATION
            if (DEBUG && $time < 100)
                $display("[MEM @%0t] Long chirp: seg=%b, addr=%d, I=%h, Q=%h",
                         $time, segment_select, long_addr,
                         long_chirp_i[long_addr], long_chirp_q[long_addr]);
            `endif
        end else begin
            // 20km mode, short-chirp disambiguation phase
            ref_i <= short_chirp_i[sample_addr];
            ref_q <= short_chirp_q[sample_addr];

            `ifdef SIMULATION
            if (DEBUG && $time < 100)
                $display("[MEM @%0t] Short chirp: addr=%d, I=%h, Q=%h",
                         $time, sample_addr,
                         short_chirp_i[sample_addr], short_chirp_q[sample_addr]);
            `endif
        end
    end
end

// ---- mem_ready — async reset (control path, not BRAM output) ----
always @(posedge clk or negedge reset_n) begin
    if (!reset_n)
        mem_ready <= 1'b0;
    else
        mem_ready <= mem_request;
end

endmodule
