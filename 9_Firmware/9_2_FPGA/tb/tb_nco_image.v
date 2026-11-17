`timescale 1ns / 1ps
// tb_nco_image: image rejection of nco_400m_enhanced.
//
// The NCO's output L[k] = cos + j*sin should be a pure e^{j*theta[k]}. Any component at e^{-j*theta[k]} mirrors
// the DDC's input around the LO: a tone 1.5625 MHz above 120 MHz also comes out 1.5625 MHz below it. The bench
// correlates (cos + j*sin) with e^{-j*theta} (wanted) and e^{+j*theta} (image) over windows that hold a whole
// number of periods of 2*theta, so the two references are orthogonal, and requires image/wanted < -70 dB in
// every window:
//   test 1: 120 MHz LO (FTW 0x4CCCCCCD, 0.3 cycles per sample), 4 windows from reset;
//   test 2: FTW 0x4CCCCCCD + 64, 2 windows, then a reset in mid-operation and 2 more: the accumulator drifts
//           inside a truncation cell, so the windows see different truncation patterns;
//   test 3: FTW 0x03000000 (3/256 cycles per sample), 2 windows: every LUT address and all four quadrants.
// Before the fix: test 3 about -18 dB (quadrants 2 and 3 folded the wrong way), tests 1 and 2 -46 to -61 dB
// (integer-step LUT and periodic phase truncation).
module tb_nco_image;
    localparam real PI     = 3.14159265358979323846;
    localparam real LIM_DB = -70.0;

    reg         clk = 1'b0;
    reg         reset_n = 1'b0;
    reg  [31:0] ftw = 32'h4CCCCCCD;
    wire signed [15:0] sin_out, cos_out;
    wire        dds_ready;
    always #1.25 clk = ~clk;

    nco_400m_enhanced dut (
        .clk_400m(clk), .reset_n(reset_n), .frequency_tuning_word(ftw), .phase_valid(1'b1),
        .phase_offset(16'h0000), .sin_out(sin_out), .cos_out(cos_out), .dds_ready(dds_ready)
    );

    integer k, w, fails;
    real    f, th, c, s, a_re, a_im, b_re, b_im, a_mag, img_db, worst_db;

    task run_window(input integer n, input [8*32-1:0] label);
        begin
            a_re = 0.0; a_im = 0.0; b_re = 0.0; b_im = 0.0;
            for (k = 0; k < n; k = k + 1) begin
                @(posedge clk);
                th = 2.0 * PI * f * k;
                c  = $itor(cos_out);
                s  = $itor(sin_out);
                a_re = a_re + c * $cos(th) + s * $sin(th);
                a_im = a_im + s * $cos(th) - c * $sin(th);
                b_re = b_re + c * $cos(th) - s * $sin(th);
                b_im = b_im + s * $cos(th) + c * $sin(th);
            end
            a_mag  = $sqrt(a_re * a_re + a_im * a_im) / n;
            img_db = 20.0 * $log10(($sqrt(b_re * b_re + b_im * b_im) / n + 1.0e-9) / a_mag);
            if (img_db > worst_db) worst_db = img_db;
            if (a_mag > 32000.0 && img_db < LIM_DB)
                $display("[PASS] %0s: |wanted| %0.1f, image %0.1f dB", label, a_mag, img_db);
            else begin
                $display("[FAIL] %0s: |wanted| %0.1f, image %0.1f dB (limit %0.1f dB)", label, a_mag, img_db, LIM_DB);
                fails = fails + 1;
            end
        end
    endtask

    initial begin
        fails = 0; worst_db = -300.0;
        repeat (8) @(posedge clk);
        #1 reset_n = 1'b1;
        wait (dds_ready);
        repeat (8) @(posedge clk);

        f = 0.3;
        for (w = 0; w < 4; w = w + 1) run_window(40000, "120 MHz LO");

        @(posedge clk); #1;
        ftw = 32'h4CCCCCCD + 32'd64;
        f   = (32'h4CCCCCCD + 64) / 4294967296.0;
        repeat (16) @(posedge clk);
        for (w = 0; w < 2; w = w + 1) run_window(40000, "120 MHz LO + 6 Hz");

        // reset in the middle of operation: the accumulator and the dither LFSR restart together
        @(posedge clk); #1 reset_n = 1'b0;
        repeat (3) @(posedge clk);
        #1 reset_n = 1'b1;
        wait (dds_ready);
        repeat (8) @(posedge clk);
        for (w = 0; w < 2; w = w + 1) run_window(40000, "120 MHz LO + 6 Hz, after reset");

        @(posedge clk); #1;
        ftw = 32'h03000000;
        f   = 3.0 / 256.0;
        repeat (16) @(posedge clk);
        for (w = 0; w < 2; w = w + 1) run_window(40960, "3/256 cycle, all quadrants");

        $display("tb_nco_image: worst image %0.1f dB, %0d failure(s)", worst_db, fails);
        $finish;
    end
endmodule
