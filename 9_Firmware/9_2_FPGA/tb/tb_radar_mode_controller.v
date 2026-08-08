`timescale 1ns / 1ps

module tb_radar_mode_controller;

    // ── Parameters ─────────────────────────────────────────────
    localparam CLK_PERIOD = 10.0;  // 100 MHz

    // Use much shorter timing for simulation (100x faster)
    localparam SIM_LONG_CHIRP   = 30;
    localparam SIM_LONG_LISTEN  = 137;
    localparam SIM_GUARD        = 175;
    localparam SIM_SHORT_CHIRP  = 5;
    localparam SIM_SHORT_LISTEN = 175;

    // Use small scan size for simulation
    localparam SIM_CHIRPS     = 4;
    localparam SIM_ELEVATIONS = 3;
    localparam SIM_AZIMUTHS   = 2;

    // ── Signals ────────────────────────────────────────────────
    reg         clk;
    reg         reset_n;
    reg  [1:0]  mode;
    reg         frame_gate;
    reg         stm32_new_chirp;
    reg         stm32_new_elevation;
    reg         stm32_new_azimuth;
    reg         trigger;

    // Gap 2: Runtime-configurable timing inputs
    reg  [15:0] cfg_long_chirp_cycles;
    reg  [15:0] cfg_long_listen_cycles;
    reg  [15:0] cfg_guard_cycles;
    reg  [15:0] cfg_short_chirp_cycles;
    reg  [15:0] cfg_short_listen_cycles;
    reg  [5:0]  cfg_chirps_per_elev;

    wire        use_long_chirp;
    wire        mc_new_chirp;
    wire        mc_new_elevation;
    wire        mc_new_azimuth;
    wire [5:0]  chirp_count;
    wire [5:0]  elevation_count;
    wire [5:0]  azimuth_count;
    wire        scanning;
    wire        scan_complete;

    // ── Test bookkeeping ───────────────────────────────────────
    integer pass_count;
    integer fail_count;
    integer test_num;
    integer csv_file;
    integer i;

    // Edge detection helpers for auto-scan counting
    reg mc_new_chirp_prev;
    reg mc_new_elevation_prev;
    reg mc_new_azimuth_prev;
    reg scanning_prev;
    integer chirp_toggles;
    integer elevation_toggles;
    integer azimuth_toggles;
    integer scan_completes;

    // Saved values for toggle checks
    reg saved_mc_new_chirp;
    reg saved_mc_new_elevation;
    reg saved_mc_new_azimuth;

    // ── Clock ──────────────────────────────────────────────────
    always #(CLK_PERIOD/2) clk = ~clk;

    // ── DUT ────────────────────────────────────────────────────
    radar_mode_controller #(
        .CHIRPS_PER_ELEVATION  (SIM_CHIRPS),
        .ELEVATIONS_PER_AZIMUTH(SIM_ELEVATIONS),
        .AZIMUTHS_PER_SCAN     (SIM_AZIMUTHS),
        .LONG_CHIRP_CYCLES     (SIM_LONG_CHIRP),
        .LONG_LISTEN_CYCLES    (SIM_LONG_LISTEN),
        .GUARD_CYCLES          (SIM_GUARD),
        .SHORT_CHIRP_CYCLES    (SIM_SHORT_CHIRP),
        .SHORT_LISTEN_CYCLES   (SIM_SHORT_LISTEN)
    ) uut (
        .clk                (clk),
        .reset_n            (reset_n),
        .mode               (mode),
        .frame_gate         (frame_gate),
        .stm32_new_chirp    (stm32_new_chirp),
        .stm32_new_elevation(stm32_new_elevation),
        .stm32_new_azimuth  (stm32_new_azimuth),
        .trigger            (trigger),
        // Gap 2: Runtime-configurable timing inputs
        .cfg_long_chirp_cycles  (cfg_long_chirp_cycles),
        .cfg_long_listen_cycles (cfg_long_listen_cycles),
        .cfg_guard_cycles       (cfg_guard_cycles),
        .cfg_short_chirp_cycles (cfg_short_chirp_cycles),
        .cfg_short_listen_cycles(cfg_short_listen_cycles),
        .cfg_chirps_per_elev    (cfg_chirps_per_elev),
        // Outputs
        .use_long_chirp     (use_long_chirp),
        .mc_new_chirp       (mc_new_chirp),
        .mc_new_elevation   (mc_new_elevation),
        .mc_new_azimuth     (mc_new_azimuth),
        .chirp_count        (chirp_count),
        .elevation_count    (elevation_count),
        .azimuth_count      (azimuth_count),
        .scanning           (scanning),
        .scan_complete      (scan_complete)
    );

    // ── Check task ─────────────────────────────────────────────
    task check;
        input cond;
        input [511:0] label;
        begin
            test_num = test_num + 1;
            if (cond) begin
                $display("[PASS] Test %0d: %0s", test_num, label);
                pass_count = pass_count + 1;
            end else begin
                $display("[FAIL] Test %0d: %0s", test_num, label);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // ── Helper: apply reset ────────────────────────────────────
    task apply_reset;
        begin
            reset_n             = 0;
            mode                = 2'b11;  // reserved = safe idle
            frame_gate          = 0;
            stm32_new_chirp     = 0;
            stm32_new_elevation = 0;
            stm32_new_azimuth   = 0;
            trigger             = 0;
            // Gap 2: Set cfg_* to simulation parameter defaults
            cfg_long_chirp_cycles  = SIM_LONG_CHIRP;
            cfg_long_listen_cycles = SIM_LONG_LISTEN;
            cfg_guard_cycles       = SIM_GUARD;
            cfg_short_chirp_cycles = SIM_SHORT_CHIRP;
            cfg_short_listen_cycles = SIM_SHORT_LISTEN;
            cfg_chirps_per_elev    = SIM_CHIRPS;
            repeat (4) @(posedge clk);
            reset_n = 1;
            @(posedge clk); #1;
        end
    endtask

    // ── Helper: pulse the frame gate (1 cycle, race-free) ─────
    // The gate must stay high when the DUT samples it. Deasserting with a
    // blocking assignment at the sampling edge races with the DUT's always
    // block (iverilog process order is unspecified) — the #1 after the edge
    // guarantees the DUT sampled gate=1 before it is deasserted.
    task pulse_gate;
        begin
            @(posedge clk);
            frame_gate = 1;
            @(posedge clk); #1;
            frame_gate = 0;
            @(posedge clk); #1;
        end
    endtask

    // ── Stimulus ───────────────────────────────────────────────
    initial begin
        $dumpfile("tb_radar_mode_controller.vcd");
        $dumpvars(0, tb_radar_mode_controller);

        clk        = 0;
        pass_count = 0;
        fail_count = 0;
        test_num   = 0;

        // ════════════════════════════════════════════════════════
        // TEST GROUP 1: Reset behaviour
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 1: Reset Behaviour ---");
        apply_reset;

        reset_n = 0;
        repeat (4) @(posedge clk); #1;
        check(use_long_chirp === 1'b1,     "use_long_chirp=1 after reset");
        check(mc_new_chirp === 1'b0,       "mc_new_chirp=0 after reset");
        check(mc_new_elevation === 1'b0,   "mc_new_elevation=0 after reset");
        check(mc_new_azimuth === 1'b0,     "mc_new_azimuth=0 after reset");
        check(chirp_count === 6'd0,        "chirp_count=0 after reset");
        check(elevation_count === 6'd0,    "elevation_count=0 after reset");
        check(azimuth_count === 6'd0,      "azimuth_count=0 after reset");
        check(scanning === 1'b0,           "scanning=0 after reset");
        check(scan_complete === 1'b0,      "scan_complete=0 after reset");
        reset_n = 1;
        @(posedge clk); #1;

        // ════════════════════════════════════════════════════════
        // TEST GROUP 2: STM32 pass-through mode (mode 00)
        // The DUT uses XOR toggle detection: when stm32_new_chirp
        // changes from its previous value, the DUT detects it.
        // We toggle-and-hold (don't pulse) to get exactly one detection.
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 2: STM32 Pass-through (mode 00) ---");
        apply_reset;
        mode = 2'b00;
        @(posedge clk); #1;

        // Save current mc_new_chirp
        saved_mc_new_chirp = mc_new_chirp;

        // Toggle stm32_new_chirp (0→1, hold at 1)
        stm32_new_chirp = 1'b1;
        // Wait 2 cycles: 1 for prev register update, 1 for XOR→main FSM
        @(posedge clk); @(posedge clk); #1;

        check(mc_new_chirp !== saved_mc_new_chirp,
              "mc_new_chirp toggles on stm32 chirp change");
        check(chirp_count === 6'd1, "chirp_count incremented to 1");

        // Toggle again (1→0, hold at 0) — second chirp
        saved_mc_new_chirp = mc_new_chirp;
        stm32_new_chirp = 1'b0;
        @(posedge clk); @(posedge clk); #1;

        check(mc_new_chirp !== saved_mc_new_chirp,
              "mc_new_chirp toggles again");
        check(chirp_count === 6'd2, "chirp_count incremented to 2");

        // Toggle stm32_new_elevation (0→1, hold)
        saved_mc_new_elevation = mc_new_elevation;
        stm32_new_elevation = 1'b1;
        @(posedge clk); @(posedge clk); #1;

        check(mc_new_elevation !== saved_mc_new_elevation,
              "mc_new_elevation toggles on stm32 elevation change");
        check(chirp_count === 6'd0,
              "chirp_count resets on elevation toggle");
        check(elevation_count === 6'd1,
              "elevation_count incremented to 1");

        // Toggle stm32_new_azimuth (0→1, hold)
        saved_mc_new_azimuth = mc_new_azimuth;
        stm32_new_azimuth = 1'b1;
        @(posedge clk); @(posedge clk); #1;

        check(mc_new_azimuth !== saved_mc_new_azimuth,
              "mc_new_azimuth toggles on stm32 azimuth change");
        check(elevation_count === 6'd0,
              "elevation_count resets on azimuth toggle");
        check(azimuth_count === 6'd1,
              "azimuth_count incremented to 1");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 3: Gated auto-scan mode (mode 01) — full scan
        // Frames are started by frame_gate pulses (mirrors TX bursts).
        // Re-gate on each frame end to complete the full scan.
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 3: Gated Auto-scan (mode 01) — Full Scan ---");
        apply_reset;
        mode = 2'b01;

        csv_file = $fopen("rmc_autoscan.csv", "w");
        $fwrite(csv_file, "cycle,chirp,elevation,azimuth,long_chirp,scanning,scan_complete\n");

        mc_new_chirp_prev     = 0;
        mc_new_elevation_prev = 0;
        mc_new_azimuth_prev   = 0;
        scanning_prev         = 0;
        chirp_toggles     = 0;
        elevation_toggles = 0;
        azimuth_toggles   = 0;
        scan_completes    = 0;

        // Check: gated auto-scan must NOT start without a gate
        @(posedge clk); #1;
        check(scanning === 1'b0, "Scanning stays idle without frame gate");

        // Fire the first gate — frame starts
        pulse_gate;
        check(scanning === 1'b1, "Scanning starts on frame gate");

        // Run enough cycles to complete one full scan, re-gating on each
        // frame end (gated auto-scan returns to S_IDLE between frames)
        for (i = 0; i < 15000; i = i + 1) begin
            @(posedge clk); #1;

            // Count the scan_complete pulse BEFORE the re-gate below:
            // pulse_gate consumes 3 clock edges, and the pulse is only
            // visible for 1-2 edges after the S_ADVANCE edge.
            if (scan_complete)
                scan_completes = scan_completes + 1;

            // Frame ended (scanning falling edge) → fire the next gate
            if (!scanning && scanning_prev) begin
                pulse_gate;
            end

            if (mc_new_chirp !== mc_new_chirp_prev)
                chirp_toggles = chirp_toggles + 1;
            if (mc_new_elevation !== mc_new_elevation_prev)
                elevation_toggles = elevation_toggles + 1;
            if (mc_new_azimuth !== mc_new_azimuth_prev)
                azimuth_toggles = azimuth_toggles + 1;
            if (scan_complete)
                scan_completes = scan_completes + 1;

            mc_new_chirp_prev     = mc_new_chirp;
            mc_new_elevation_prev = mc_new_elevation;
            mc_new_azimuth_prev   = mc_new_azimuth;
            scanning_prev         = scanning;

            if (i % 100 == 0) begin
                $fwrite(csv_file, "%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",
                        i, chirp_count, elevation_count, azimuth_count,
                        use_long_chirp, scanning, scan_complete);
            end
        end

        $fclose(csv_file);

        $display("  Chirp toggles:     %0d (expected %0d)",
                 chirp_toggles, SIM_CHIRPS * SIM_ELEVATIONS * SIM_AZIMUTHS);
        $display("  Elevation toggles: %0d", elevation_toggles);
        $display("  Azimuth toggles:   %0d", azimuth_toggles);
        $display("  Scan completes:    %0d", scan_completes);

        check(chirp_toggles >= SIM_CHIRPS * SIM_ELEVATIONS * SIM_AZIMUTHS,
              "At least 24 chirp toggles in full scan");
        check(scan_completes >= 1,
              "At least 1 scan completion detected");
        check(elevation_toggles >= SIM_AZIMUTHS,
              "Elevation toggles >= number of azimuths");
        check(azimuth_toggles >= 1,
              "Azimuth toggles >= 1");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 4: Gated auto-scan chirp timing
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 4: Chirp Timing Sequence ---");
        apply_reset;
        mode = 2'b01;

        pulse_gate;
        @(posedge clk); #1;
        check(use_long_chirp === 1'b1, "Starts with long chirp after gate");

        repeat (SIM_LONG_CHIRP / 2) @(posedge clk);
        #1;
        check(use_long_chirp === 1'b1, "Still long chirp midway");

        // Wait through remainder of long chirp + long listen + guard
        repeat (SIM_LONG_CHIRP / 2 + SIM_LONG_LISTEN + SIM_GUARD) @(posedge clk);
        #1;

        // Now should be in short chirp phase (with 1-2 cycles margin)
        repeat (2) @(posedge clk); #1;
        check(use_long_chirp === 1'b0, "Switches to short chirp after guard");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 5: Single-chirp mode (mode 10)
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 5: Single-chirp Mode (mode 10) ---");
        apply_reset;
        mode = 2'b10;

        repeat (10) @(posedge clk); #1;
        check(scanning === 1'b0, "Single mode: idle without trigger");

        saved_mc_new_chirp = mc_new_chirp;

        // Pulse trigger (rising edge detection)
        trigger = 1'b1;
        @(posedge clk); #1;
        trigger = 1'b0;

        repeat (2) @(posedge clk); #1;
        check(scanning === 1'b1, "Single mode: scanning after trigger");
        check(use_long_chirp === 1'b1, "Single mode: uses long chirp");
        check(mc_new_chirp !== saved_mc_new_chirp,
              "Single mode: mc_new_chirp toggled");

        // Wait for chirp to complete
        repeat (SIM_LONG_CHIRP + SIM_LONG_LISTEN + 10) @(posedge clk); #1;
        check(scanning === 1'b0, "Single mode: returns to idle after chirp");

        // No activity without trigger
        saved_mc_new_chirp = mc_new_chirp;
        repeat (100) @(posedge clk); #1;
        check(mc_new_chirp === saved_mc_new_chirp,
              "Single mode: no activity without trigger");

        // Second trigger
        saved_mc_new_chirp = mc_new_chirp;
        trigger = 1'b1;
        @(posedge clk); #1;
        trigger = 1'b0;
        repeat (3) @(posedge clk); #1;
        check(mc_new_chirp !== saved_mc_new_chirp,
              "Single mode: 2nd trigger works");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 6: Reserved mode (mode 11) — stays idle
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 6: Reserved Mode (mode 11) ---");
        apply_reset;
        mode = 2'b11;

        repeat (200) @(posedge clk); #1;
        check(scanning === 1'b0, "Reserved mode: stays idle");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 7: Mode switching
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 7: Mode Switching ---");
        apply_reset;
        mode = 2'b01;  // Gated auto-scan

        pulse_gate;
        repeat (100) @(posedge clk); #1;
        check(scanning === 1'b1, "Auto mode: scanning (gated)");

        mode = 2'b11;
        repeat (10) @(posedge clk); #1;
        check(scanning === 1'b0, "Switching to reserved: stops scanning");

        mode = 2'b10;
        repeat (10) @(posedge clk); #1;
        check(scanning === 1'b0, "Single mode after switch: idle");

        trigger = 1'b1;
        @(posedge clk); #1;
        trigger = 1'b0;
        repeat (3) @(posedge clk); #1;
        check(scanning === 1'b1, "Single mode after switch: triggers OK");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 8: STM32 mode — chirp count wrapping
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 8: STM32 Chirp Count Wrapping ---");
        apply_reset;
        mode = 2'b00;
        @(posedge clk); #1;

        // Toggle chirp SIM_CHIRPS times (toggle-and-hold each time)
        for (i = 0; i < SIM_CHIRPS; i = i + 1) begin
            stm32_new_chirp = ~stm32_new_chirp;  // toggle and hold
            @(posedge clk); @(posedge clk); #1;   // wait for detection
        end

        $display("  chirp_count after %0d toggles: %0d (expect 0)",
                 SIM_CHIRPS, chirp_count);
        check(chirp_count === 6'd0,
              "chirp_count wraps after CHIRPS_PER_ELEVATION toggles");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 9: STM32 mode — full scan completion
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 9: STM32 Full Scan Completion ---");
        apply_reset;
        mode = 2'b00;
        @(posedge clk); #1;

        scan_completes = 0;

        // Toggle azimuth SIM_AZIMUTHS times
        for (i = 0; i < SIM_AZIMUTHS; i = i + 1) begin
            stm32_new_azimuth = ~stm32_new_azimuth;
            @(posedge clk); #1;
            if (scan_complete) scan_completes = scan_completes + 1;
            @(posedge clk); #1;
            if (scan_complete) scan_completes = scan_completes + 1;
        end

        $display("  scan_complete pulses: %0d (expect 1)", scan_completes);
        check(scan_completes == 1, "scan_complete pulses once after full azimuth sweep");
        check(azimuth_count === 6'd0, "azimuth_count wraps to 0 after full scan");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 10: Reset Mid-Scan
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 10: Reset Mid-Scan ---");
        apply_reset;
        mode = 2'b01;  // gated auto-scan
        pulse_gate;

        // Wait ~200 cycles (partway through first chirp)
        repeat (200) @(posedge clk); #1;
        check(scanning === 1'b1, "Mid-scan: scanning=1 before reset");

        // Assert reset for 4 cycles
        reset_n = 0;
        repeat (4) @(posedge clk); #1;

        // Verify state during reset
        check(scanning === 1'b0,           "Mid-scan reset: scanning=0");
        check(chirp_count === 6'd0,        "Mid-scan reset: chirp_count=0");
        check(elevation_count === 6'd0,    "Mid-scan reset: elevation_count=0");
        check(azimuth_count === 6'd0,      "Mid-scan reset: azimuth_count=0");
        check(use_long_chirp === 1'b1,     "Mid-scan reset: use_long_chirp=1");
        check(mc_new_chirp === 1'b0,       "Mid-scan reset: mc_new_chirp=0");
        check(mc_new_elevation === 1'b0,   "Mid-scan reset: mc_new_elevation=0");
        check(mc_new_azimuth === 1'b0,     "Mid-scan reset: mc_new_azimuth=0");

        // Release reset
        reset_n = 1;
        @(posedge clk); #1;

        // ════════════════════════════════════════════════════════
        // TEST GROUP 11: Mode-Switch State Leakage
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 11: Mode-Switch State Leakage ---");
        apply_reset;
        mode = 2'b01;  // gated auto-scan
        pulse_gate;

        // Run for ~500 cycles
        repeat (500) @(posedge clk); #1;
        check(scanning === 1'b1, "Leakage: scanning=1 during gated auto-scan");

        // Switch to reserved mode (11) — forces scan_state=S_IDLE
        mode = 2'b11;
        repeat (10) @(posedge clk); #1;
        check(scanning === 1'b0, "Leakage: scanning=0 in reserved mode");

        // Switch back to gated auto-scan (01) — stays idle until a gate
        mode = 2'b01;
        @(posedge clk); #1;
        check(scanning === 1'b0, "Leakage: stays idle in auto-scan until gate");
        pulse_gate;
        check(scanning === 1'b1, "Leakage: auto-scan restarts cleanly on gate (scanning=1)");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 12: Simultaneous STM32 Toggle Events
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 12: Simultaneous STM32 Toggle Events ---");
        apply_reset;
        mode = 2'b00;
        @(posedge clk); #1;

        // Save current toggle outputs
        saved_mc_new_chirp     = mc_new_chirp;
        saved_mc_new_elevation = mc_new_elevation;

        // Toggle BOTH stm32_new_chirp AND stm32_new_elevation at the same time
        stm32_new_chirp     = 1'b1;
        stm32_new_elevation = 1'b1;
        // Wait 2 cycles for XOR detection
        @(posedge clk); @(posedge clk); #1;

        check(mc_new_chirp !== saved_mc_new_chirp,
              "Simultaneous: mc_new_chirp toggled");
        check(mc_new_elevation !== saved_mc_new_elevation,
              "Simultaneous: mc_new_elevation toggled");
        // Elevation toggle resets chirp_count (last-write-wins in RTL)
        check(chirp_count === 6'd0,
              "Simultaneous: chirp_count=0 (elevation resets it)");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 13: Single-Chirp Mode — Multiple Rapid Triggers
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 13: Single-Chirp Multiple Rapid Triggers ---");
        apply_reset;
        mode = 2'b10;
        @(posedge clk); #1;

        saved_mc_new_chirp = mc_new_chirp;

        // First trigger — should start a chirp
        trigger = 1'b1;
        @(posedge clk); #1;
        trigger = 1'b0;
        repeat (2) @(posedge clk); #1;
        check(scanning === 1'b1, "Rapid trigger: first trigger starts chirp");
        check(mc_new_chirp !== saved_mc_new_chirp,
              "Rapid trigger: mc_new_chirp toggled on first trigger");

        // Save chirp state after first trigger
        saved_mc_new_chirp = mc_new_chirp;

        // Send another trigger while chirp is still active (FSM not in S_IDLE)
        trigger = 1'b1;
        @(posedge clk); #1;
        trigger = 1'b0;
        repeat (2) @(posedge clk); #1;
        check(scanning === 1'b1, "Rapid trigger: still scanning (didn't restart)");
        check(mc_new_chirp === saved_mc_new_chirp,
              "Rapid trigger: second trigger ignored (mc_new_chirp unchanged)");

        // Wait for chirp to complete (long_chirp + long_listen total)
        repeat (SIM_LONG_CHIRP + SIM_LONG_LISTEN + 20) @(posedge clk); #1;
        check(scanning === 1'b0, "Rapid trigger: chirp completed, back to idle");

        // Now trigger again — this should work
        saved_mc_new_chirp = mc_new_chirp;
        trigger = 1'b1;
        @(posedge clk); #1;
        trigger = 1'b0;
        repeat (2) @(posedge clk); #1;
        check(scanning === 1'b1, "Rapid trigger: third trigger works after idle");
        check(mc_new_chirp !== saved_mc_new_chirp,
              "Rapid trigger: mc_new_chirp toggled on third trigger");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 14: Gated Auto-Scan Counter Verification
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 14: Auto-Scan Counter Verification ---");
        apply_reset;
        mode = 2'b01;  // gated auto-scan

        mc_new_chirp_prev     = 0;
        scanning_prev         = 0;
        chirp_toggles  = 0;
        scan_completes = 0;

        // The first chirp toggle happens on the gate's S_IDLE→S_LONG_CHIRP
        // transition. Fire the first gate, then sample and count from there.
        pulse_gate;
        // After this, scan_state is S_LONG_CHIRP and mc_new_chirp has already
        // toggled once. Record its value as prev so we can count from here.
        mc_new_chirp_prev = mc_new_chirp;
        chirp_toggles = 1;  // count the initial toggle

        // Run until first scan_complete, re-gating on each frame end.
        // Total chirps = 4*3*2 = 24, each chirp ~523 cycles → 6 frames of
        // 4 chirps ≈ 12552 cycles, add margin.
        for (i = 0; i < 14000; i = i + 1) begin
            @(posedge clk); #1;
            if (scan_complete)
                scan_completes = scan_completes + 1;
            // Stop BEFORE counting the toggle that starts the next frame
            if (scan_completes >= 1)
                i = 14000;  // break
            else begin
                // Frame ended → fire the next gate (frame n+1 starts)
                if (!scanning && scanning_prev) begin
                    pulse_gate;
                end
                if (mc_new_chirp !== mc_new_chirp_prev)
                    chirp_toggles = chirp_toggles + 1;
                mc_new_chirp_prev = mc_new_chirp;
                scanning_prev = scanning;
            end
        end

        $display("  Total chirp toggles: %0d (expected 24)", chirp_toggles);
        $display("  Scan completes:      %0d (expected 1)", scan_completes);

        // At scan_complete, the DUT wraps all counters and returns to S_IDLE
        // (gated — the next frame only starts on a new frame_gate).
        check(scan_completes == 1, "Counter verify: exactly 1 scan_complete");
        // The full-scan-complete branch resets all counters to 0:
        check(chirp_count === 6'd0,     "Counter verify: chirp_count=0 at scan_complete");
        check(elevation_count === 6'd0, "Counter verify: elevation_count=0 at scan_complete");
        check(azimuth_count === 6'd0,   "Counter verify: azimuth_count=0 at scan_complete");
        check(chirp_toggles == SIM_CHIRPS * SIM_ELEVATIONS * SIM_AZIMUTHS,
              "Counter verify: exactly 24 chirp toggles");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 15: STM32 Mode — Counter Persistence
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 15: STM32 Mode Counter Persistence ---");
        apply_reset;
        mode = 2'b00;
        @(posedge clk); #1;

        // Toggle chirp 3 times
        for (i = 0; i < 3; i = i + 1) begin
            stm32_new_chirp = ~stm32_new_chirp;
            @(posedge clk); @(posedge clk); #1;
        end

        $display("  chirp_count after 3 toggles: %0d (expect 3)", chirp_count);
        check(chirp_count === 6'd3, "Persistence: chirp_count=3 after 3 toggles");

        // Switch to reserved mode (11) — does NOT reset counters
        mode = 2'b11;
        repeat (10) @(posedge clk); #1;

        $display("  chirp_count in reserved mode: %0d (expect 3)", chirp_count);
        check(chirp_count === 6'd3, "Persistence: chirp_count=3 in reserved mode");

        // Switch back to STM32 mode (00)
        mode = 2'b00;
        @(posedge clk); #1;

        $display("  chirp_count after returning to STM32: %0d (expect 3)", chirp_count);
        check(chirp_count === 6'd3, "Persistence: chirp_count=3 after mode roundtrip");

        // Toggle chirp once more — should wrap (3+1=4=CHIRPS, wraps to 0)
        stm32_new_chirp = ~stm32_new_chirp;
        @(posedge clk); @(posedge clk); #1;

        $display("  chirp_count after 4th toggle: %0d (expect 0)", chirp_count);
        check(chirp_count === 6'd0, "Persistence: chirp_count wraps to 0 at 4th toggle");

        // ════════════════════════════════════════════════════════
        // TEST GROUP 16: Runtime Timing Reconfiguration (Gap 2)
        // Verify that changing cfg_* mid-simulation changes timing.
        // We halve the long chirp duration and verify the chirp
        // completes in fewer cycles.
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 16: Runtime Timing Reconfiguration (Gap 2) ---");
        apply_reset;
        mode = 2'b01;  // gated auto-scan

        // Fire the gate so the frame starts (S_IDLE -> S_LONG_CHIRP)
        pulse_gate;
        @(posedge clk); #1;
        check(scanning === 1'b1, "Reconfig: auto-scan started (gated)");
        check(use_long_chirp === 1'b1, "Reconfig: starts with long chirp");

        // Wait ~half the default long chirp time to confirm we're still in S_LONG_CHIRP
        repeat (SIM_LONG_CHIRP / 2) @(posedge clk); #1;
        check(uut.scan_state === 3'd1, "Reconfig: still in S_LONG_CHIRP at midpoint");

        // Now change cfg_long_chirp_cycles to a much shorter value mid-scan.
        // The timer is already at ~SIM_LONG_CHIRP/2, so setting cycles to
        // (SIM_LONG_CHIRP/2 - 2) means the timer already exceeds the new limit
        // and the FSM will advance on the next cycle.
        cfg_long_chirp_cycles = SIM_LONG_CHIRP / 2 - 2;
        repeat (2) @(posedge clk); #1;
        // The FSM should have transitioned past S_LONG_CHIRP
        check(uut.scan_state !== 3'd1, "Reconfig: FSM left S_LONG_CHIRP after shortening cycles");

        // Restore default and verify scan continues
        cfg_long_chirp_cycles = SIM_LONG_CHIRP;
        repeat (10) @(posedge clk); #1;
        check(scanning === 1'b1, "Reconfig: scan continues after restoring default");

        // Test runtime chirps_per_elev change:
        // Reset and set chirps_per_elev to 2 (instead of default 4)
        apply_reset;
        cfg_chirps_per_elev = 6'd2;
        mode = 2'b01;  // gated auto-scan

        mc_new_chirp_prev = 0;
        mc_new_elevation_prev = 0;
        chirp_toggles = 0;
        elevation_toggles = 0;

        pulse_gate;
        @(posedge clk); #1;
        mc_new_chirp_prev = mc_new_chirp;
        mc_new_elevation_prev = mc_new_elevation;
        chirp_toggles = 1;  // initial toggle

        // Run enough cycles for a few chirps + elevation advance
        // With 2 chirps/elev: each chirp ~342 cycles (30+137+175+5+175)
        // 2 chirps = ~684 cycles, then elevation advance
        for (i = 0; i < 2000; i = i + 1) begin
            @(posedge clk); #1;
            if (mc_new_chirp !== mc_new_chirp_prev)
                chirp_toggles = chirp_toggles + 1;
            if (mc_new_elevation !== mc_new_elevation_prev)
                elevation_toggles = elevation_toggles + 1;
            mc_new_chirp_prev = mc_new_chirp;
            mc_new_elevation_prev = mc_new_elevation;
        end

        $display("  chirp_toggles=%0d elevation_toggles=%0d (cfg_chirps_per_elev=2)",
                 chirp_toggles, elevation_toggles);
        // With 2 chirps/elev, we should get elevation toggles at every 2 chirps
        check(elevation_toggles >= 1,
              "Reconfig: elevation advances with cfg_chirps_per_elev=2");
        // Verify the ratio: chirp_toggles should be ~2x elevation_toggles
        // (first elevation has 2 chirps, then toggle. Second has 2 chirps, then toggle, etc.)
        check(chirp_toggles >= 2 * elevation_toggles,
              "Reconfig: chirp/elevation ratio consistent with cfg_chirps_per_elev=2");

        // Restore defaults
        cfg_chirps_per_elev = SIM_CHIRPS;

        // ════════════════════════════════════════════════════════
        // TEST GROUP 17: Frame Gating Contract (reliability feature)
        // RX frames must start only on frame_gate and must not run
        // between gates (MCU-gated TX bursts).
        // ════════════════════════════════════════════════════════
        $display("\n--- Test Group 17: Frame Gating Contract ---");
        apply_reset;
        mode = 2'b01;

        // G17a: no activity without a gate
        repeat (100) @(posedge clk); #1;
        check(scanning === 1'b0, "G17a: no scanning without frame gate");

        // G17b/c: first gate starts a frame and toggles mc_new_chirp
        saved_mc_new_chirp = mc_new_chirp;
        pulse_gate;
        check(scanning === 1'b1, "G17b: scanning starts on frame gate");
        check(mc_new_chirp !== saved_mc_new_chirp,
              "G17c: mc_new_chirp toggled at frame start");

        // G17d: frame completes and returns to gated idle
        // Frame = 4 chirps x (30+137+175+5+175) = 2088 cycles + margin
        repeat (2400) @(posedge clk); #1;
        check(scanning === 1'b0, "G17d: returns to gated idle after frame");

        // G17e: no chirps between gates
        saved_mc_new_chirp = mc_new_chirp;
        repeat (500) @(posedge clk); #1;
        check(mc_new_chirp === saved_mc_new_chirp,
              "G17e: no chirp activity between gates");

        // G17f/g: a second gate starts a second frame
        saved_mc_new_chirp = mc_new_chirp;
        pulse_gate;
        check(scanning === 1'b1, "G17f: second gate starts second frame");
        check(mc_new_chirp !== saved_mc_new_chirp,
              "G17g: mc_new_chirp toggled on second frame");

        // ════════════════════════════════════════════════════════
        // Summary
        // ════════════════════════════════════════════════════════
        $display("");
        $display("========================================");
        $display("  RADAR MODE CONTROLLER RESULTS");
        $display("  PASSED: %0d / %0d", pass_count, test_num);
        $display("  FAILED: %0d / %0d", fail_count, test_num);
        if (fail_count == 0)
            $display("  ** ALL TESTS PASSED **");
        else
            $display("  ** SOME TESTS FAILED **");
        $display("========================================");
        $display("");

        #100;
        $finish;
    end

endmodule
