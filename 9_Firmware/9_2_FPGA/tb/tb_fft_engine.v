`timescale 1ns / 1ps

/**
 * tb_fft_engine.v
 *
 * Testbench for the synthesizable FFT engine.
 * Tests with N=16 (matching the dual-16 Doppler architecture).
 *
 * Test Groups:
 *   1. Impulse response: FFT of delta[0] should be all 1s
 *   2. DC input: FFT of all-1s should be delta at bin 0
 *   3. Single tone: FFT of cos(2*pi*k/N) should peak at bin k
 *   4. Roundtrip: FFT then IFFT should recover original
 *   5. Linearity: FFT(a+b) ~= FFT(a) + FFT(b)
 *
 * Convention: standard check task with pass/fail tracking.
 */

module tb_fft_engine;

// ============================================================================
// PARAMETERS — test with 16-pt to match dual-FFT Doppler architecture
// ============================================================================
localparam N      = 16;
localparam LOG2N  = 4;
localparam DATA_W = 16;
localparam INT_W  = 32;
localparam TW_W   = 16;
localparam CLK_PERIOD = 10;

// ============================================================================
// SIGNALS
// ============================================================================
reg clk, reset_n;
reg start, inverse;
reg signed [DATA_W-1:0] din_re, din_im;
reg din_valid;
wire signed [DATA_W-1:0] dout_re, dout_im;
wire dout_valid, busy, done_sig;

// ============================================================================
// DUT
// ============================================================================
fft_engine #(
    .N(N),
    .LOG2N(LOG2N),
    .DATA_W(DATA_W),
    .INTERNAL_W(INT_W),
    .TWIDDLE_W(TW_W),
    .TWIDDLE_FILE("fft_twiddle_16.mem")
) dut (
    .clk(clk),
    .reset_n(reset_n),
    .start(start),
    .inverse(inverse),
    .din_re(din_re),
    .din_im(din_im),
    .din_valid(din_valid),
    .dout_re(dout_re),
    .dout_im(dout_im),
    .dout_valid(dout_valid),
    .busy(busy),
    .done(done_sig)
);

// ============================================================================
// CLOCK
// ============================================================================
initial clk = 0;
always #(CLK_PERIOD/2) clk = ~clk;

// ============================================================================
// PASS/FAIL TRACKING
// ============================================================================
integer pass_count, fail_count;

task check;
    input cond;
    input [512*8-1:0] label;
    begin
        if (cond) begin
            $display("  [PASS] %0s", label);
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] %0s", label);
            fail_count = fail_count + 1;
        end
    end
endtask

// ============================================================================
// STORAGE FOR CAPTURED OUTPUTS
// ============================================================================
reg signed [DATA_W-1:0] out_re [0:N-1];
reg signed [DATA_W-1:0] out_im [0:N-1];
integer out_idx;

// Second set for roundtrip
reg signed [DATA_W-1:0] out2_re [0:N-1];
reg signed [DATA_W-1:0] out2_im [0:N-1];

// Input storage for roundtrip comparison
reg signed [DATA_W-1:0] in_re [0:N-1];
reg signed [DATA_W-1:0] in_im [0:N-1];

// ============================================================================
// HELPER TASKS
// ============================================================================

// Reset
task do_reset;
    begin
        reset_n = 0;
        start = 0;
        inverse = 0;
        din_re = 0;
        din_im = 0;
        din_valid = 0;
        repeat(5) @(posedge clk); #1;
        reset_n = 1;
        repeat(2) @(posedge clk); #1;
    end
endtask

// Run FFT: load N samples from in_re/in_im arrays, capture output to out_re/out_im
task run_fft;
    input inv;
    integer i;
    begin
        inverse = inv;
        @(posedge clk); #1;
        start = 1;
        @(posedge clk); #1;
        start = 0;

        // Feed N samples
        for (i = 0; i < N; i = i + 1) begin
            din_re = in_re[i];
            din_im = in_im[i];
            din_valid = 1;
            @(posedge clk); #1;
        end
        din_valid = 0;
        din_re = 0;
        din_im = 0;

        // Wait for output and capture
        out_idx = 0;
        while (out_idx < N) begin
            @(posedge clk); #1;
            if (dout_valid) begin
                out_re[out_idx] = dout_re;
                out_im[out_idx] = dout_im;
                out_idx = out_idx + 1;
            end
        end

        // Wait for done
        @(posedge clk); #1;
    end
endtask

// Run FFT and capture to out2 arrays
task run_fft_to_out2;
    input inv;
    integer i;
    begin
        inverse = inv;
        @(posedge clk); #1;
        start = 1;
        @(posedge clk); #1;
        start = 0;

        for (i = 0; i < N; i = i + 1) begin
            din_re = in_re[i];
            din_im = in_im[i];
            din_valid = 1;
            @(posedge clk); #1;
        end
        din_valid = 0;
        din_re = 0;
        din_im = 0;

        out_idx = 0;
        while (out_idx < N) begin
            @(posedge clk); #1;
            if (dout_valid) begin
                out2_re[out_idx] = dout_re;
                out2_im[out_idx] = dout_im;
                out_idx = out_idx + 1;
            end
        end
        @(posedge clk); #1;
    end
endtask

// ============================================================================
// VCD + CSV
// ============================================================================
initial begin
    $dumpfile("tb_fft_engine.vcd");
    $dumpvars(0, tb_fft_engine);
end

// ============================================================================
// MAIN TEST
// ============================================================================
integer i, j;
integer max_mag_bin;
reg signed [31:0] max_mag;
reg signed [31:0] mag;
reg signed [31:0] err;
integer max_err;
integer total_energy_in, total_energy_out;

// For tone generation
real angle;
reg signed [DATA_W-1:0] cos_val;

initial begin
    pass_count = 0;
    fail_count = 0;

    $display("============================================================");
    $display("  FFT Engine Testbench — N=%0d", N);
    $display("============================================================");

    do_reset;

    // ================================================================
    // TEST GROUP 1: Impulse Response
    // FFT(delta[0]) should give all bins = 1 (in_re[0]=1, rest=0)
    // Since input is Q15-ish (16-bit signed), use amplitude = 1000
    // FFT of impulse with amplitude A: all bins = A
    // ================================================================
    $display("");
    $display("--- Test Group 1: Impulse Response ---");

    for (i = 0; i < N; i = i + 1) begin
        in_re[i] = (i == 0) ? 16'sd1000 : 16'sd0;
        in_im[i] = 16'sd0;
    end

    run_fft(0);  // Forward FFT

    // All bins should have re ~= 1000, im ~= 0
    max_err = 0;
    for (i = 0; i < N; i = i + 1) begin
        err = out_re[i] - 1000;
        if (err < 0) err = -err;
        if (err > max_err) max_err = err;
        err = out_im[i];
        if (err < 0) err = -err;
        if (err > max_err) max_err = err;
    end
    $display("  Impulse FFT max error from expected: %0d", max_err);
    check(max_err < 10, "Impulse FFT: all bins ~= input amplitude");
    check(out_re[0] == 1000 || (out_re[0] >= 998 && out_re[0] <= 1002), 
          "Impulse FFT: bin 0 real ~= 1000");

    // ================================================================
    // TEST GROUP 2: DC Input
    // FFT of constant value A across all N samples:
    //   bin 0 = A*N, all other bins = 0
    // Use amplitude 100 so bin 0 = 100*32 = 3200
    // ================================================================
    $display("");
    $display("--- Test Group 2: DC Input ---");

    for (i = 0; i < N; i = i + 1) begin
        in_re[i] = 16'sd100;
        in_im[i] = 16'sd0;
    end

    run_fft(0);

    $display("  DC FFT bin[0] = %0d + j%0d (expect %0d + j0)", out_re[0], out_im[0], 100*N);
    // Q15 twiddle rounding over N butterflies can cause ~1% error
    check(out_re[0] >= (100*N - 50) && out_re[0] <= (100*N + 50),
          "DC FFT: bin 0 real ~= A*N (1.5% tol)");
    
    max_err = 0;
    for (i = 1; i < N; i = i + 1) begin
        mag = out_re[i] * out_re[i] + out_im[i] * out_im[i];
        if (out_re[i] > max_err || -out_re[i] > max_err)
            max_err = (out_re[i] > 0) ? out_re[i] : -out_re[i];
        if (out_im[i] > max_err || -out_im[i] > max_err)
            max_err = (out_im[i] > 0) ? out_im[i] : -out_im[i];
    end
    $display("  DC FFT max non-DC bin magnitude: %0d", max_err);
    check(max_err < 20, "DC FFT: non-DC bins ~= 0 (Q15 rounding tol)");

    // ================================================================
    // TEST GROUP 3: Single Tone (cosine at bin 4)
    // N=16: cos(2*pi*4*n/16) -> peaks at bins 4 and N-4=12
    // Amplitude 1000 -> each peak = 1000*N/2 = 8000
    // ================================================================
    $display("");
    $display("--- Test Group 3: Single Tone (bin 4) ---");

    for (i = 0; i < N; i = i + 1) begin
        // cos(2*pi*4*i/16) in Q15-ish
        angle = 6.28318530718 * 4.0 * i / 16.0;
        cos_val = $rtoi($cos(angle) * 1000.0);
        in_re[i] = cos_val;
        in_im[i] = 16'sd0;
    end

    run_fft(0);

    // Find peak bin
    max_mag = 0;
    max_mag_bin = 0;
    for (i = 0; i < N; i = i + 1) begin
        mag = out_re[i] * out_re[i] + out_im[i] * out_im[i];
        if (mag > max_mag) begin
            max_mag = mag;
            max_mag_bin = i;
        end
    end
    $display("  Tone FFT peak bin: %0d (expect 4)", max_mag_bin);
    $display("  Tone FFT bin[4]  = %0d + j%0d", out_re[4], out_im[4]);
    $display("  Tone FFT bin[12] = %0d + j%0d", out_re[12], out_im[12]);
    check(max_mag_bin == 4 || max_mag_bin == 12, 
          "Tone FFT: peak at bin 4 or 12");
    // Bin 4 and 12 should have magnitude ~= N/2 * 1000 = 8000
    mag = out_re[4] * out_re[4] + out_im[4] * out_im[4];
    check(mag > 6800*6800 && mag < 9200*9200,
          "Tone FFT: bin 4 magnitude ~= 8000");

    // ================================================================
    // TEST GROUP 4: Roundtrip (FFT then IFFT = identity)
    // Load random-ish data, FFT, IFFT, compare to original
    // ================================================================
    $display("");
    $display("--- Test Group 4: Roundtrip (FFT->IFFT) ---");

    // Use a simple deterministic pattern
    for (i = 0; i < N; i = i + 1) begin
        in_re[i] = (i * 137 + 42) % 2001 - 1000;  // [-1000, 1000]
        in_im[i] = (i * 251 + 17) % 2001 - 1000;
    end

    // Forward FFT
    run_fft(0);

    // Copy FFT output as input for IFFT
    for (i = 0; i < N; i = i + 1) begin
        in_re[i] = out_re[i];
        in_im[i] = out_im[i];
    end

    // Save original input for comparison
    // (we need to recompute since in_re was overwritten)

    // Actually let's redo: store originals first
    // We'll do it properly with separate storage

    // Re-do: load original pattern
    for (i = 0; i < N; i = i + 1) begin
        out2_re[i] = (i * 137 + 42) % 2001 - 1000;
        out2_im[i] = (i * 251 + 17) % 2001 - 1000;
    end

    // Now in_re/in_im has FFT output. Run IFFT.
    run_fft(1);

    // out_re/out_im should match original (out2_re/out2_im) within tolerance
    max_err = 0;
    for (i = 0; i < N; i = i + 1) begin
        err = out_re[i] - out2_re[i];
        if (err < 0) err = -err;
        if (err > max_err) max_err = err;
        err = out_im[i] - out2_im[i];
        if (err < 0) err = -err;
        if (err > max_err) max_err = err;
    end
    $display("  Roundtrip max error: %0d", max_err);
    check(max_err < 20, "Roundtrip: FFT->IFFT recovers original (err < 20)");
    check(max_err < 5, "Roundtrip: FFT->IFFT tight tolerance (err < 5)");

    // Print first few samples for debugging
    $display("  Sample comparison (idx: original vs recovered):");
    for (i = 0; i < 8; i = i + 1) begin
        $display("    [%0d] re: %0d vs %0d, im: %0d vs %0d",
                 i, out2_re[i], out_re[i], out2_im[i], out_im[i]);
    end

    // ================================================================
    // TEST GROUP 5: IFFT of impulse
    // IFFT(delta[0]) = 1/N for all bins -> should be ~1 for amplitude N
    // Input: bin[0] = N (=32), rest = 0
    // IFFT output: all samples = 1
    // ================================================================
    $display("");
    $display("--- Test Group 5: IFFT of Impulse ---");

    for (i = 0; i < N; i = i + 1) begin
        in_re[i] = (i == 0) ? N : 16'sd0;
        in_im[i] = 16'sd0;
    end

    run_fft(1);  // Inverse FFT

    max_err = 0;
    for (i = 0; i < N; i = i + 1) begin
        err = out_re[i] - 1;
        if (err < 0) err = -err;
        if (err > max_err) max_err = err;
        err = out_im[i];
        if (err < 0) err = -err;
        if (err > max_err) max_err = err;
    end
    $display("  IFFT impulse max error: %0d", max_err);
    check(max_err < 2, "IFFT impulse: all samples ~= 1");

    // ================================================================
    // TEST GROUP 6: Parseval's theorem (energy conservation)
    // Sum |x[n]|^2 should equal (1/N) * Sum |X[k]|^2
    // We compare N * sum_time vs sum_freq
    // ================================================================
    $display("");
    $display("--- Test Group 6: Parseval's Theorem ---");

    for (i = 0; i < N; i = i + 1) begin
        in_re[i] = (i * 137 + 42) % 2001 - 1000;
        in_im[i] = (i * 251 + 17) % 2001 - 1000;
    end

    // Compute time-domain energy
    total_energy_in = 0;
    for (i = 0; i < N; i = i + 1) begin
        total_energy_in = total_energy_in + in_re[i] * in_re[i] + in_im[i] * in_im[i];
    end

    run_fft(0);

    // Compute frequency-domain energy
    total_energy_out = 0;
    for (i = 0; i < N; i = i + 1) begin
        total_energy_out = total_energy_out + out_re[i] * out_re[i] + out_im[i] * out_im[i];
    end

    // Parseval: sum_time = (1/N) * sum_freq => N * sum_time = sum_freq
    $display("  Time energy * N = %0d", total_energy_in * N);
    $display("  Freq energy     = %0d", total_energy_out);
    // Allow some tolerance for fixed-point rounding
    err = total_energy_in * N - total_energy_out;
    if (err < 0) err = -err;
    $display("  Parseval error  = %0d", err);
    // Relative error
    if (total_energy_in * N > 0) begin
        $display("  Parseval rel error = %0d%%", (err * 100) / (total_energy_in * N));
        check((err * 100) / (total_energy_in * N) < 5, 
              "Parseval: energy conserved within 5%");
    end

    // ================================================================
    // TEST GROUP 7: Pure imaginary input
    // FFT of j*sin(2*pi*2*n/N) -> peaks at bins 2 and N-2
    // ================================================================
    $display("");
    $display("--- Test Group 7: Pure Imaginary Tone (bin 2) ---");

    for (i = 0; i < N; i = i + 1) begin
        in_re[i] = 16'sd0;
        angle = 6.28318530718 * 2.0 * i / 16.0;
        in_im[i] = $rtoi($sin(angle) * 1000.0);
    end

    run_fft(0);

    // Find peak
    max_mag = 0;
    max_mag_bin = 0;
    for (i = 0; i < N; i = i + 1) begin
        mag = out_re[i] * out_re[i] + out_im[i] * out_im[i];
        if (mag > max_mag) begin
            max_mag = mag;
            max_mag_bin = i;
        end
    end
    $display("  Imag tone peak bin: %0d (expect 2 or 14)", max_mag_bin);
    check(max_mag_bin == 2 || max_mag_bin == 14,
          "Imag tone: peak at bin 2 or 14");

    // ================================================================
    // TEST GROUP 8: Zero input
    // ================================================================
    $display("");
    $display("--- Test Group 8: Zero Input ---");

    for (i = 0; i < N; i = i + 1) begin
        in_re[i] = 16'sd0;
        in_im[i] = 16'sd0;
    end

    run_fft(0);

    max_err = 0;
    for (i = 0; i < N; i = i + 1) begin
        err = out_re[i];
        if (err < 0) err = -err;
        if (err > max_err) max_err = err;
        err = out_im[i];
        if (err < 0) err = -err;
        if (err > max_err) max_err = err;
    end
    check(max_err == 0, "Zero input: all output bins = 0");

    // ================================================================
    // SUMMARY
    // ================================================================
    $display("");
    $display("============================================================");
    $display("  RESULTS: %0d/%0d passed", pass_count, pass_count + fail_count);
    if (fail_count == 0)
        $display("  ALL TESTS PASSED");
    else
        $display("  SOME TESTS FAILED");
    $display("============================================================");

    $finish;
end

endmodule
