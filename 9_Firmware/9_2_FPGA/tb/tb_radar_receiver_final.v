`timescale 1ns / 1ps
// ============================================================================
// tb_radar_receiver_final.v -- P0 Integration Test for radar_receiver_final
//
// Tests the full RX pipeline from ADC input to Doppler output:
//   ad9484_interface (stub) -> CDC -> DDC -> ddc_input_interface
//     -> matched_filter_multi_segment -> range_bin_decimator
//     -> doppler_processor_optimized -> doppler_output
//
// ============================================================================
// TWO MODES (compile-time define):
//
//   1. GOLDEN_GENERATE mode  (-DGOLDEN_GENERATE):
//      Dumps all Doppler output samples to golden reference files.
//      Run once on known-good RTL:
//        iverilog -g2001 -DSIMULATION -DGOLDEN_GENERATE -o tb_golden_gen.vvp \
//          <src files> tb/tb_radar_receiver_final.v
//        mkdir -p tb/golden
//        vvp tb_golden_gen.vvp
//
//   2. Default mode (no GOLDEN_GENERATE):
//      Loads golden files, compares each Doppler output against reference,
//      and runs physics-based bounds checks.
//        iverilog -g2001 -DSIMULATION -o tb_radar_receiver_final.vvp \
//          <src files> tb/tb_radar_receiver_final.v
//        vvp tb_radar_receiver_final.vvp
//
// PREREQUISITES:
//   - The directory tb/golden/ must exist before running either mode.
//     Create it with: mkdir -p tb/golden
//
// TAP POINTS:
//   Tap 1 (DDC output)     - bounds checking only (CDC jitter -> non-deterministic)
//     Signals: dut.ddc_out_i [17:0], dut.ddc_out_q [17:0], dut.ddc_valid_i
//   Tap 2 (Doppler output) - golden compared (deterministic after MF buffering)
//     Signals: doppler_output[31:0], doppler_valid, doppler_bin[4:0],
//              range_bin_out[5:0]
//
// Golden file: tb/golden/golden_doppler.mem
//   2048 entries of 32-bit hex, indexed by range_bin*32 + doppler_bin
//
// Strategy:
//   - Uses behavioral stub for ad9484_interface_400m (no Xilinx primitives)
//   - Overrides radar_mode_controller timing params for fast simulation
//   - Feeds 120 MHz tone at ADC input (IF frequency -> DDC passband)
//   - Verifies structural correctness + golden comparison + bounds checks
//
// Convention: check task, VCD dump, CSV output, pass/fail summary
// ============================================================================

module tb_radar_receiver_final;

// ============================================================================
// CLOCK AND RESET
// ============================================================================
reg clk_100m;       // 100 MHz system clock
reg clk_400m;       // 400 MHz ADC clock
reg reset_n;

// 100 MHz: period = 10 ns
initial clk_100m = 0;
always #5 clk_100m = ~clk_100m;

// 400 MHz: period = 2.5 ns
initial clk_400m = 0;
always #1.25 clk_400m = ~clk_400m;

// ============================================================================
// ADC STIMULUS
// ============================================================================
// Feed a 120 MHz tone (IF frequency) sampled at 400 MHz
// Phase increment per sample: 120/400 * 65536 = 19660.8
// This produces a strong DC component after DDC downconversion
reg [7:0] adc_data;
reg [15:0] phase_acc;  // 16-bit phase accumulator for precision
localparam [15:0] PHASE_INC = 16'd19661;  // 120/400 * 65536

always @(posedge clk_400m or negedge reset_n) begin
    if (!reset_n) begin
        phase_acc <= 16'd0;
        adc_data <= 8'd128;  // Mid-scale
    end else begin
        phase_acc <= phase_acc + PHASE_INC;
        // Use phase_acc[15:8] directly as pseudo-sinusoidal data
        // A sawtooth/triangle wave has energy at IF -- good enough for integration test
        adc_data <= phase_acc[15:8];
    end
end

// ============================================================================
// CHIRP COUNTER (external input to DUT)
// ============================================================================
// In the real system, this comes from the transmitter. For the test,
// we increment it on each mc_new_chirp toggle from the mode controller.
// Access the internal signal via hierarchical reference.
reg [5:0] chirp_counter;
reg mc_new_chirp_prev;

// Frame-start pulse (frame_gate): mirrors the real transmitter's
// new_chirp_frame.  The RX mode controller is now GATED (mode 01 waits in
// S_IDLE for frame_gate), so the gate must be generated independently:
//   - one pulse after reset release to start the first frame,
//   - then one pulse every time a frame ends (scanning falling edge),
//     which makes frames run back-to-back like the real TX bursts.
reg tx_frame_start;
reg scanning_prev;
reg initial_gate_done;

always @(posedge clk_100m or negedge reset_n) begin
    if (!reset_n) begin
        chirp_counter <= 6'd0;
        mc_new_chirp_prev <= 1'b0;
        tx_frame_start <= 1'b0;
        scanning_prev <= 1'b0;
        initial_gate_done <= 1'b0;
    end else begin
        mc_new_chirp_prev <= dut.mc_new_chirp;
        if (dut.mc_new_chirp != mc_new_chirp_prev) begin
            chirp_counter <= chirp_counter + 6'd1;
        end

        tx_frame_start <= 1'b0;
        if (!initial_gate_done) begin
            // First gate shortly after reset release starts frame 1
            initial_gate_done <= 1'b1;
            tx_frame_start <= 1'b1;
        end else if (!dut.rmc.scanning && scanning_prev) begin
            // Frame ended (S_IDLE) → next gate starts the next frame
            tx_frame_start <= 1'b1;
        end
        scanning_prev <= dut.rmc.scanning;
    end
end

// ============================================================================
// DUT INSTANTIATION
// ============================================================================
wire [31:0] doppler_output;
wire doppler_valid;
wire [4:0] doppler_bin;
wire [5:0] range_bin_out;

radar_receiver_final dut (
    .clk(clk_100m),
    .reset_n(reset_n),

    // ADC "LVDS" -- stub treats adc_d_p as single-ended data
    .adc_d_p(adc_data),
    .adc_d_n(~adc_data),       // Complement (ignored by stub)
    .adc_dco_p(clk_400m),      // 400 MHz clock
    .adc_dco_n(~clk_400m),     // Complement (ignored by stub)
    .adc_pwdn(),

    .chirp_counter(chirp_counter),
    .tx_frame_start(tx_frame_start),

    .doppler_output(doppler_output),
    .doppler_valid(doppler_valid),
    .doppler_bin(doppler_bin),
    .range_bin(range_bin_out),

    // Range profile outputs (unused in this TB)
    .range_profile_i_out(),
    .range_profile_q_out(),
    .range_profile_valid_out(),

    // Host command inputs (Gap 4) — default auto-scan, no trigger
    .host_mode(2'b01),
    .host_trigger(1'b0),

    // Gap 2: Host-configurable chirp timing — match defparam overrides below
    .host_long_chirp_cycles(16'd500),
    .host_long_listen_cycles(16'd2000),
    .host_guard_cycles(16'd500),
    .host_short_chirp_cycles(16'd50),
    .host_short_listen_cycles(16'd1000),
    .host_chirps_per_elev(6'd32),

    // Fix 3: digital gain control — pass-through for golden reference
    .host_gain_shift(4'd0),
    // CFAR: frame-complete output (not used in this TB)
    .doppler_frame_done_out()
);

// ============================================================================
// OVERRIDE TIMING PARAMETERS via defparam
// ============================================================================
// Reduce radar_mode_controller timing to keep simulation tractable.
// Real values: LONG_CHIRP=3000, LONG_LISTEN=13700, GUARD=17540,
//              SHORT_CHIRP=50, SHORT_LISTEN=17450  (total ~51740 per chirp)
// Need enough DDC samples to fill MF buffer (896) plus latency buffer (3187).
// At ~1 DDC sample per sys_clk, we need at least ~5000 sys_clk per chirp.
// Use moderately reduced values: ~5000 cycles per chirp pair
defparam dut.rmc.LONG_CHIRP_CYCLES   = 500;
defparam dut.rmc.LONG_LISTEN_CYCLES  = 2000;
defparam dut.rmc.GUARD_CYCLES        = 500;
defparam dut.rmc.SHORT_CHIRP_CYCLES  = 50;
defparam dut.rmc.SHORT_LISTEN_CYCLES = 1000;

// ============================================================================
// TEST INFRASTRUCTURE
// ============================================================================
integer pass_count;
integer fail_count;
integer total_tests;

task check;
    input cond;
    input [512*8-1:0] label;
    begin
        total_tests = total_tests + 1;
        if (cond) begin
            pass_count = pass_count + 1;
            $display("[PASS %0d] %0s", total_tests, label);
        end else begin
            fail_count = fail_count + 1;
            $display("[FAIL %0d] %0s", total_tests, label);
        end
    end
endtask

// ============================================================================
// GOLDEN MEMORY DECLARATIONS AND LOAD/STORE LOGIC
// ============================================================================
localparam GOLDEN_ENTRIES   = 2048;  // 64 range bins * 32 Doppler bins
localparam GOLDEN_TOLERANCE = 2;     // +/- 2 LSB tolerance for comparison

reg [31:0] golden_doppler [0:2047];

// -- Golden comparison tracking --
integer golden_match_count;
integer golden_mismatch_count;
integer golden_max_err_i;
integer golden_max_err_q;
integer golden_compare_count;

`ifdef GOLDEN_GENERATE
    // In generate mode, we just initialize the array to X/0
    // and fill it as outputs arrive
    integer gi;
    initial begin
        for (gi = 0; gi < GOLDEN_ENTRIES; gi = gi + 1)
            golden_doppler[gi] = 32'd0;
        golden_match_count   = 0;
        golden_mismatch_count = 0;
        golden_max_err_i     = 0;
        golden_max_err_q     = 0;
        golden_compare_count = 0;
    end
`else
    // In comparison mode, load the golden reference
    initial begin
        $readmemh("tb/golden/golden_doppler.mem", golden_doppler);
        golden_match_count   = 0;
        golden_mismatch_count = 0;
        golden_max_err_i     = 0;
        golden_max_err_q     = 0;
        golden_compare_count = 0;
    end
`endif

// ============================================================================
// DDC ENERGY ACCUMULATOR (Bounds Check B1)
// ============================================================================
// Accumulate I^2 + Q^2 for all DDC valid samples. 64-bit to avoid overflow.
// DDC outputs are 18-bit signed -> squared max ~ 2^34, sum of many -> need 64-bit.
reg [63:0] ddc_energy_acc;
integer    ddc_sample_count;

initial begin
    ddc_energy_acc  = 64'd0;
    ddc_sample_count = 0;
end

always @(posedge clk_100m) begin
    if (reset_n && dut.ddc_valid_i) begin
        ddc_energy_acc <= ddc_energy_acc
            + ($signed(dut.ddc_out_i) * $signed(dut.ddc_out_i))
            + ($signed(dut.ddc_out_q) * $signed(dut.ddc_out_q));
        ddc_sample_count = ddc_sample_count + 1;
    end
end

// ============================================================================
// DOPPLER OUTPUT CAPTURE, GOLDEN COMPARISON, AND DUPLICATE DETECTION
// ============================================================================
integer doppler_output_count;
integer doppler_frame_count;
reg [63:0] range_bin_seen;     // Bitmap: which range bins appeared
reg [31:0] doppler_bin_seen;   // Bitmap: which Doppler bins appeared
integer nonzero_output_count;
reg [31:0] first_doppler_time; // Cycle when first doppler_valid appears
reg first_doppler_seen;

// Per-frame tracking
integer frame_output_count;
reg frame_done_prev;

// CSV output
integer csv_fd;

// Duplicate detection: one-hot bitmap per (range_bin, doppler_bin)
// 64 range bins x 32 doppler bins = 2048 bits -> use an array of 64 x 32-bit regs
reg [31:0] index_seen [0:63];
integer dup_count;

// Bounds check B2: Doppler energy tracking per range bin
// For each range bin, track peak |I|+|Q| across all 32 Doppler bins
// and total energy. Verifies pipeline computes non-trivial Doppler spectra.
reg [31:0] peak_dbin_mag [0:63]; // max |I|+|Q| across all Doppler bins
reg [31:0] total_dbin_energy [0:63]; // sum of |I|+|Q| across all 32 Doppler bins
integer b2_init_idx;

initial begin
    doppler_output_count = 0;
    doppler_frame_count = 0;
    range_bin_seen = 64'd0;
    doppler_bin_seen = 32'd0;
    nonzero_output_count = 0;
    first_doppler_seen = 0;
    first_doppler_time = 0;
    frame_output_count = 0;
    frame_done_prev = 0;
    dup_count = 0;

    for (b2_init_idx = 0; b2_init_idx < 64; b2_init_idx = b2_init_idx + 1) begin
        index_seen[b2_init_idx]      = 32'd0;
        peak_dbin_mag[b2_init_idx]   = 32'd0;
        total_dbin_energy[b2_init_idx] = 32'd0;
    end

    csv_fd = $fopen("tb/cosim/rx_final_doppler_out.csv", "w");
    if (csv_fd) $fdisplay(csv_fd, "cycle,range_bin,doppler_bin,output_hex");
end

// Monitor doppler outputs -- only after reset released
always @(posedge clk_100m) begin
    if (reset_n && doppler_valid) begin : doppler_capture_block
        // ---- Signed intermediates for golden comparison ----
        reg signed [16:0] actual_i, actual_q;
        reg signed [16:0] expected_i, expected_q;
        reg signed [16:0] err_i_signed, err_q_signed;
        integer abs_err_i, abs_err_q;
        integer gidx;
        reg [31:0] expected_val;
        // ---- Magnitude intermediates for B2 ----
        reg signed [16:0] mag_i_signed, mag_q_signed;
        integer mag_i, mag_q, mag_sum;

        doppler_output_count = doppler_output_count + 1;
        frame_output_count = frame_output_count + 1;

        // Track which bins we've seen
        if (range_bin_out < 64)
            range_bin_seen = range_bin_seen | (64'd1 << range_bin_out);
        if (doppler_bin < 32)
            doppler_bin_seen = doppler_bin_seen | (32'd1 << doppler_bin);

        // Track non-zero outputs
        if (doppler_output != 32'd0)
            nonzero_output_count = nonzero_output_count + 1;

        // Record first output time
        if (!first_doppler_seen) begin
            first_doppler_seen = 1;
            first_doppler_time = $time;
            $display("[INFO] First doppler_valid at time %0t", $time);
        end

        // CSV logging
        if (csv_fd)
            $fdisplay(csv_fd, "%0t,%0d,%0d,%08h", $time, range_bin_out, doppler_bin, doppler_output);

        // Progress reporting (every 256 outputs)
        if ((doppler_output_count % 256) == 0)
            $display("[INFO] %0d doppler outputs so far (t=%0t)", doppler_output_count, $time);

        // ---- Golden index computation ----
        gidx = range_bin_out * 32 + doppler_bin;

        // ---- Duplicate detection (B5) ----
        if (range_bin_out < 64 && doppler_bin < 32) begin
            if (index_seen[range_bin_out][doppler_bin]) begin
                dup_count = dup_count + 1;
                if (dup_count <= 10)
                    $display("[WARN] Duplicate index: rbin=%0d dbin=%0d (count=%0d)",
                             range_bin_out, doppler_bin, dup_count);
            end
            index_seen[range_bin_out] = index_seen[range_bin_out] | (32'd1 << doppler_bin);
        end

        // ---- Bounds check B2: Doppler energy tracking ----
        mag_i_signed = $signed(doppler_output[15:0]);
        mag_q_signed = $signed(doppler_output[31:16]);
        mag_i = (mag_i_signed < 0) ? -mag_i_signed : mag_i_signed;
        mag_q = (mag_q_signed < 0) ? -mag_q_signed : mag_q_signed;
        mag_sum = mag_i + mag_q;

        if (range_bin_out < 64) begin
            total_dbin_energy[range_bin_out] = total_dbin_energy[range_bin_out] + mag_sum;
            if (mag_sum > peak_dbin_mag[range_bin_out])
                peak_dbin_mag[range_bin_out] = mag_sum;
        end

`ifdef GOLDEN_GENERATE
        // ---- GOLDEN GENERATE: store output ----
        if (gidx < GOLDEN_ENTRIES)
            golden_doppler[gidx] = doppler_output;
`else
        // ---- GOLDEN COMPARE: check against reference ----
        if (gidx < GOLDEN_ENTRIES) begin
            expected_val = golden_doppler[gidx];

            actual_i   = $signed(doppler_output[15:0]);
            actual_q   = $signed(doppler_output[31:16]);
            expected_i = $signed(expected_val[15:0]);
            expected_q = $signed(expected_val[31:16]);

            err_i_signed = actual_i - expected_i;
            err_q_signed = actual_q - expected_q;

            abs_err_i = (err_i_signed < 0) ? -err_i_signed : err_i_signed;
            abs_err_q = (err_q_signed < 0) ? -err_q_signed : err_q_signed;

            golden_compare_count = golden_compare_count + 1;

            if (abs_err_i > golden_max_err_i) golden_max_err_i = abs_err_i;
            if (abs_err_q > golden_max_err_q) golden_max_err_q = abs_err_q;

            if (abs_err_i <= GOLDEN_TOLERANCE && abs_err_q <= GOLDEN_TOLERANCE) begin
                golden_match_count = golden_match_count + 1;
            end else begin
                golden_mismatch_count = golden_mismatch_count + 1;
                if (golden_mismatch_count <= 20)
                    $display("[MISMATCH] idx=%0d rbin=%0d dbin=%0d actual=%08h expected=%08h err_i=%0d err_q=%0d",
                             gidx, range_bin_out, doppler_bin,
                             doppler_output, expected_val,
                             abs_err_i, abs_err_q);
            end
        end
`endif
    end

    // Track frame completions via doppler_proc -- only after reset
    if (reset_n && dut.doppler_frame_done && !frame_done_prev) begin
        doppler_frame_count = doppler_frame_count + 1;
        $display("[INFO] Doppler frame %0d complete: %0d outputs (t=%0t)",
                 doppler_frame_count, frame_output_count, $time);
        frame_output_count = 0;
    end
    frame_done_prev = dut.doppler_frame_done;
end

// ============================================================================
// PROGRESS MONITOR -- pipeline stage activity
// ============================================================================
reg [31:0] ddc_valid_count;
reg [31:0] mf_valid_count;
reg [31:0] range_decim_count;
reg [31:0] range_data_valid_count;

initial begin
    ddc_valid_count = 0;
    mf_valid_count = 0;
    range_decim_count = 0;
    range_data_valid_count = 0;
end

always @(posedge clk_100m) begin
    if (dut.adc_valid_sync) ddc_valid_count = ddc_valid_count + 1;
    if (dut.range_valid) mf_valid_count = mf_valid_count + 1;
    if (dut.decimated_range_valid) range_decim_count = range_decim_count + 1;
    if (dut.range_data_valid) range_data_valid_count = range_data_valid_count + 1;
end

// Periodic progress dump
reg [31:0] progress_timer;
initial progress_timer = 0;
always @(posedge clk_100m) begin
    progress_timer = progress_timer + 1;
    if (progress_timer % 50000 == 0) begin
        $display("[PROGRESS t=%0t] ddc_valid=%0d mf_out=%0d range_decim=%0d doppler_out=%0d chirp=%0d",
                 $time, ddc_valid_count, mf_valid_count, range_decim_count,
                 doppler_output_count, chirp_counter);
    end
end

// ============================================================================
// MF PIPELINE DEBUG MONITOR -- track state transitions
// ============================================================================
reg [3:0] mf_state_prev;
reg [3:0] chain_state_prev;
initial begin
    mf_state_prev = 0;
    chain_state_prev = 0;
end

always @(posedge clk_100m) begin
    // Multi-segment FSM state changes
    if (dut.mf_dual.state != mf_state_prev) begin
        $display("[MF_DBG t=%0t] multi_seg state: %0d -> %0d (seg=%0d, wr_ptr=%0d, rd_ptr=%0d, samples=%0d)",
                 $time, mf_state_prev, dut.mf_dual.state,
                 dut.mf_dual.current_segment, dut.mf_dual.buffer_write_ptr,
                 dut.mf_dual.buffer_read_ptr, dut.mf_dual.chirp_samples_collected);
        mf_state_prev = dut.mf_dual.state;
    end
    // Processing chain state changes
    if (dut.mf_dual.m_f_p_c.state != chain_state_prev) begin
        $display("[CHAIN_DBG t=%0t] chain state: %0d -> %0d (fwd_count=%0d, out_count=%0d)",
                 $time, chain_state_prev, dut.mf_dual.m_f_p_c.state,
                 dut.mf_dual.m_f_p_c.fwd_in_count, dut.mf_dual.m_f_p_c.out_count);
        chain_state_prev = dut.mf_dual.m_f_p_c.state;
    end
    // Watch for fft_pc_valid while multi-seg is in ST_WAIT_FFT
    if (dut.mf_dual.state == 5 && dut.mf_dual.fft_pc_valid) begin
        $display("[MF_DBG t=%0t] *** fft_pc_valid=1 while in ST_WAIT_FFT! Should transition!", $time);
    end
    // Watch for fft_pc_valid while multi-seg is NOT in ST_WAIT_FFT
    if (dut.mf_dual.state != 5 && dut.mf_dual.fft_pc_valid) begin
        $display("[MF_DBG t=%0t] WARNING: fft_pc_valid=1 but multi_seg state=%0d (NOT ST_WAIT_FFT)",
                 $time, dut.mf_dual.state);
    end
end

// ============================================================================
// MAIN TEST SEQUENCE
// ============================================================================
// Simulation timeout calculation:
// 1. DDC pipeline fill: ~4 sys_clk cycles
// 2. MF overlap-save buffer fill: 896 valid DDC samples
// 3. Latency buffer priming: 3187 valid_in assertions
// 4. 1024 MF outputs -> range_bin_decimator -> 64 decimated outputs
// 5. 32 chirps of decimated data -> Doppler FFT
//
// With shortened mode controller timing (~600 cycles per chirp pair),
// DDC output rate depends on how many 400MHz samples per chirp period
// produce valid 100MHz outputs (CIC 4x decimation = ~1 per 4 clk_400m).
//
// Conservative estimate: ~500K 100MHz cycles for the full pipeline.
// ~4050 cycles/chirp x 32 chirps = ~130K, plus latency buffer priming,
// plus Doppler processing time. Set generous timeout.

localparam SIM_TIMEOUT = 2_000_000;  // 2M cycles -- full pipeline with multi-segment drain

// Maximum DDC RMS energy threshold (B1). 18-bit ADC, squared max ~2^34.
// With ~100k samples, absolute max energy ~ 100000 * 2^34 ~ 1.7e15 < 2^51.
// Set a generous ceiling that catches true overflow/garbage.
localparam [63:0] DDC_MAX_ENERGY = 64'h00FF_FFFF_FFFF_FFFF; // ~2^56

initial begin
    // VCD dump disabled for long integration test -- uncomment for debug
    // $dumpfile("tb/tb_radar_receiver_final.vcd");
    // $dumpvars(0, tb_radar_receiver_final);

    pass_count = 0;
    fail_count = 0;
    total_tests = 0;

    // ---- RESET ----
    reset_n = 0;
    #100;
    reset_n = 1;
    $display("[INFO] Reset released at t=%0t", $time);

    // ---- WAIT FOR PIPELINE ----
    // Poll until first Doppler frame completes or timeout
    begin : wait_loop
        integer wait_cycles;
        wait_cycles = 0;
        while (doppler_frame_count < 1 && wait_cycles < SIM_TIMEOUT) begin
            @(posedge clk_100m);
            wait_cycles = wait_cycles + 1;
        end
        if (doppler_frame_count >= 1) begin
            $display("[INFO] First Doppler frame completed at t=%0t", $time);
            #1000;
        end else begin
            $display("[WARN] Simulation timeout reached at t=%0t (%0d cycles)", $time, wait_cycles);
            $display("[WARN] Pipeline progress: ddc_valid=%0d mf_out=%0d range_decim=%0d doppler=%0d",
                     ddc_valid_count, mf_valid_count, range_decim_count, doppler_output_count);
        end
    end

    // ---- DUMP GOLDEN FILE (generate mode only) ----
`ifdef GOLDEN_GENERATE
    $writememh("tb/golden/golden_doppler.mem", golden_doppler);
    $display("[GOLDEN_GENERATE] Wrote tb/golden/golden_doppler.mem (%0d entries captured)",
             doppler_output_count);
`endif

    // ================================================================
    // RUN CHECKS
    // ================================================================
    $display("");
    $display("============================================================");
    $display("RADAR RECEIVER FINAL -- INTEGRATION TEST RESULTS");
    $display("============================================================");
    $display("Total doppler outputs:   %0d", doppler_output_count);
    $display("Doppler frames complete: %0d", doppler_frame_count);
    $display("Non-zero outputs:        %0d", nonzero_output_count);
    $display("DDC valid count:         %0d", ddc_valid_count);
    $display("DDC sample count (tap):  %0d", ddc_sample_count);
    $display("MF output count:         %0d", mf_valid_count);
    $display("Range decim count:       %0d", range_decim_count);
    $display("============================================================");
    $display("");

    // ================================================================
    // STRUCTURAL CHECKS (original 10 checks, kept as-is)
    // ================================================================

    // ---- CHECK S1: Pipeline activity ----
    check(ddc_valid_count > 0,
          "S1: DDC produces valid outputs (adc_valid_sync asserted)");

    // ---- CHECK S2: MF outputs appear ----
    check(mf_valid_count > 0,
          "S2: Matched filter produces outputs (range_valid asserted)");

    // ---- CHECK S3: Range decimator outputs appear ----
    check(range_decim_count > 0,
          "S3: Range bin decimator produces outputs");

    // ---- CHECK S4: Doppler outputs appear ----
    check(doppler_output_count > 0,
          "S4: Doppler processor produces outputs (doppler_valid asserted)");

    // ---- CHECK S5: Correct output count per frame (legacy: >= 2048) ----
    if (doppler_frame_count > 0) begin
        check(doppler_output_count >= 2048,
              "S5: At least 2048 doppler outputs (one full frame: 64 rbins x 32 dbins)");
    end else begin
        check(0, "S5: At least 2048 doppler outputs (NO FRAME COMPLETED)");
    end

    // ---- CHECK S6: Range bin coverage ----
    begin : count_range_bins
        integer rb_count, rb_i;
        rb_count = 0;
        for (rb_i = 0; rb_i < 64; rb_i = rb_i + 1) begin
            if (range_bin_seen[rb_i]) rb_count = rb_count + 1;
        end
        $display("[INFO] Unique range bins seen: %0d / 64", rb_count);
        check(rb_count == 64,
              "S6: All 64 range bins present in Doppler output");
    end

    // ---- CHECK S7: Doppler bin coverage ----
    begin : count_doppler_bins
        integer db_count, db_i;
        db_count = 0;
        for (db_i = 0; db_i < 32; db_i = db_i + 1) begin
            if (doppler_bin_seen[db_i]) db_count = db_count + 1;
        end
        $display("[INFO] Unique Doppler bins seen: %0d / 32", db_count);
        check(db_count == 32,
              "S7: All 32 Doppler bins present in Doppler output");
    end

    // ---- CHECK S8: Non-trivial outputs ----
    check(nonzero_output_count > 0,
          "S8: At least some Doppler outputs are non-zero");

    // ---- CHECK S9: Non-zero fraction ----
    if (doppler_output_count > 0) begin
        check(nonzero_output_count > doppler_output_count / 4,
              "S9: More than 25pct of Doppler outputs are non-zero");
    end else begin
        check(0, "S9: More than 25pct of Doppler outputs are non-zero (NO OUTPUTS)");
    end

    // ---- CHECK S10: Pipeline didn't stall ----
    check(ddc_valid_count > 100,
          "S10: DDC produced substantial output (>100 valid samples)");

    // ================================================================
    // GOLDEN COMPARISON REPORT
    // ================================================================
`ifdef GOLDEN_GENERATE
    $display("");
    $display("Golden comparison:  SKIPPED (GOLDEN_GENERATE mode)");
    $display("  Wrote golden reference with %0d Doppler samples", doppler_output_count);
`else
    $display("");
    $display("------------------------------------------------------------");
    $display("GOLDEN COMPARISON (tolerance=%0d LSB)", GOLDEN_TOLERANCE);
    $display("------------------------------------------------------------");
    $display("Golden comparison:  %0d/%0d match (tolerance=%0d LSB)",
             golden_match_count, golden_compare_count, GOLDEN_TOLERANCE);
    $display("  Mismatches: %0d (I-ch max_err=%0d, Q-ch max_err=%0d)",
             golden_mismatch_count, golden_max_err_i, golden_max_err_q);

    // CHECK G1: All golden comparisons match
    if (golden_compare_count > 0) begin
        check(golden_mismatch_count == 0,
              "G1: All Doppler outputs match golden reference within tolerance");
    end else begin
        check(0, "G1: All Doppler outputs match golden reference (NO COMPARISONS)");
    end
`endif

    // ================================================================
    // BOUNDS CHECKS (active in both modes)
    // ================================================================
    $display("");
    $display("------------------------------------------------------------");
    $display("BOUNDS CHECKS");
    $display("------------------------------------------------------------");

    // ---- B1: DDC RMS Energy ----
    $display("  DDC energy accumulator: %0d (samples=%0d)", ddc_energy_acc, ddc_sample_count);
    check(ddc_energy_acc > 64'd0,
          "B1a: DDC RMS energy > 0 (DDC is not dead)");
    check(ddc_energy_acc < DDC_MAX_ENERGY,
          "B1b: DDC RMS energy < MAX_THRESHOLD (no overflow/garbage)");

    // ---- B2: Doppler Energy Per Range Bin ----
    // Every range bin should have non-trivial Doppler energy (peak mag > 0)
    // and reasonable total energy (not degenerate). This catches a dead MF or
    // Doppler stage that produces zeros for some range bins.
    begin : b2_check_block
        integer b2_rb;
        integer nontrivial_count;
        integer min_peak, max_peak;
        nontrivial_count = 0;
        min_peak = 32'h7FFFFFFF;
        max_peak = 0;
        for (b2_rb = 0; b2_rb < 64; b2_rb = b2_rb + 1) begin
            if (peak_dbin_mag[b2_rb] > 0)
                nontrivial_count = nontrivial_count + 1;
            if (peak_dbin_mag[b2_rb] < min_peak)
                min_peak = peak_dbin_mag[b2_rb];
            if (peak_dbin_mag[b2_rb] > max_peak)
                max_peak = peak_dbin_mag[b2_rb];
        end
        $display("  Doppler peak mag: min=%0d max=%0d, non-trivial in %0d/64 range bins",
                 min_peak, max_peak, nontrivial_count);
        // All 64 range bins must have non-zero peak Doppler energy
        check(nontrivial_count == 64,
              "B2a: All range bins have non-trivial Doppler energy");
        // Peak magnitude should be bounded (not overflowing to max signed value)
        check(max_peak < 32000,
              "B2b: Peak Doppler magnitude within expected range (no overflow)");
    end

    // ---- B3: Exact Doppler Output Count ----
    $display("  Doppler output count: %0d (expected 2048)", doppler_output_count);
    check(doppler_output_count == 2048,
          "B3: Exact output count = 2048 (64 range x 32 Doppler)");

    // ---- B4: Full Range/Doppler Bin Coverage (exact) ----
    begin : b4_check_block
        integer b4_rb_count, b4_db_count, b4_i;
        b4_rb_count = 0;
        b4_db_count = 0;
        for (b4_i = 0; b4_i < 64; b4_i = b4_i + 1) begin
            if (range_bin_seen[b4_i]) b4_rb_count = b4_rb_count + 1;
        end
        for (b4_i = 0; b4_i < 32; b4_i = b4_i + 1) begin
            if (doppler_bin_seen[b4_i]) b4_db_count = b4_db_count + 1;
        end
        check(b4_rb_count == 64 && b4_db_count == 32,
              "B4: Full bin coverage: 64 range x 32 Doppler");
    end

    // ---- B5: No Duplicate Indices ----
    $display("  Duplicate (rbin, dbin) indices: %0d", dup_count);
    check(dup_count == 0,
          "B5: No duplicate (rbin, dbin) indices");

    // ================================================================
    // FINAL SUMMARY
    // ================================================================
    $display("");
    $display("============================================================");
    $display("INTEGRATION TEST -- GOLDEN COMPARISON + BOUNDS");
    $display("============================================================");
`ifdef GOLDEN_GENERATE
    $display("Mode: GOLDEN_GENERATE (reference dump, comparison skipped)");
`else
    $display("Golden comparison:  %0d/%0d match (tolerance=%0d LSB)",
             golden_match_count, golden_compare_count, GOLDEN_TOLERANCE);
    $display("  Mismatches: %0d (I-ch max_err=%0d, Q-ch max_err=%0d)",
             golden_mismatch_count, golden_max_err_i, golden_max_err_q);
`endif
    $display("Bounds checks:");
    $display("  B1: DDC RMS energy in range [%0d, %0d]",
             (ddc_energy_acc > 0) ? 1 : 0, DDC_MAX_ENERGY);
    $display("  B2: Doppler energy per range bin check");
    $display("  B3: Exact output count: %0d", doppler_output_count);
    $display("  B4: Full bin coverage");
    $display("  B5: Duplicate index count: %0d", dup_count);
    $display("============================================================");
    $display("SUMMARY: %0d / %0d tests passed", pass_count, total_tests);
    if (fail_count == 0)
        $display("ALL TESTS PASSED");
    else
        $display("SOME TESTS FAILED (%0d failures)", fail_count);
    $display("============================================================");

    if (csv_fd) $fclose(csv_fd);
    $finish;
end

endmodule
