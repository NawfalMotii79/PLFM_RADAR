`timescale 1ns / 1ps

`include "radar_params.vh"

/**
 * radar_system_top.v
 * 
 * Complete Radar System Top Module
 * Integrates:
 * - Radar Transmitter (PLFM chirp generation)
 * - Radar Receiver (ADC interface, DDC, matched filtering, Doppler processing)
 * - USB Data Interface (FT601 USB 3.0 or FT2232H USB 2.0, selected by USB_MODE)
 * 
 * Clock domains:
 * - clk_100m: System clock (100MHz)
 * - clk_120m_dac: DAC clock (120MHz)
 * - ft601_clk: USB interface clock (100MHz FT601 or 60MHz FT2232H)
 *
 * USB_MODE parameter:
 *   0 = FT601 (32-bit, USB 3.0) — 200T premium board
 *   1 = FT2232H (8-bit, USB 2.0) — 50T production board
 */

module radar_system_top (
    // System Clocks
    input wire clk_100m,                // 100MHz system clock
    input wire clk_120m_dac,             // 120MHz DAC clock
    input wire ft601_clk_in,             // FT601 clock (100MHz)
    input wire reset_n,                   // Active-low reset
    
    // ========== TRANSMITTER INTERFACES ==========
    
    // DAC Interface
    output wire [7:0] dac_data,
    output wire dac_clk,
    output wire dac_sleep,
    
    // RF Switch Control
    output wire fpga_rf_switch,
    
    // Mixer Enables
    output wire rx_mixer_en,
    output wire tx_mixer_en,
    
    // ADAR1000 Beamformer Control (via level shifters)
    output wire adar_tx_load_1, adar_rx_load_1,
    output wire adar_tx_load_2, adar_rx_load_2,
    output wire adar_tx_load_3, adar_rx_load_3,
    output wire adar_tx_load_4, adar_rx_load_4,
    output wire adar_tr_1, adar_tr_2, adar_tr_3, adar_tr_4,
    
    // Level Shifter SPI Interface (STM32F7 to ADAR1000)
    input wire stm32_sclk_3v3,
    input wire stm32_mosi_3v3,
    output wire stm32_miso_3v3,
    input wire stm32_cs_adar1_3v3, stm32_cs_adar2_3v3, 
    input wire stm32_cs_adar3_3v3, stm32_cs_adar4_3v3,
    
    output wire stm32_sclk_1v8,
    output wire stm32_mosi_1v8,
    input wire stm32_miso_1v8,
    output wire stm32_cs_adar1_1v8, stm32_cs_adar2_1v8,
    output wire stm32_cs_adar3_1v8, stm32_cs_adar4_1v8,
    
    // ========== RECEIVER INTERFACES ==========
    
    // ADC Physical Interface (LVDS)
    input wire [7:0] adc_d_p,            // ADC Data P (LVDS)
    input wire [7:0] adc_d_n,            // ADC Data N (LVDS)
    input wire adc_dco_p,                 // Data Clock Output P (400MHz LVDS)
    input wire adc_dco_n,                 // Data Clock Output N (400MHz LVDS)
    // Audit F-0.1: AD9484 OR (overrange) LVDS pair
    input wire adc_or_p,
    input wire adc_or_n,
    output wire adc_pwdn,                  // ADC Power Down
    
    // ========== STM32 CONTROL INTERFACES ==========
    
    // Chirp/Beam Control (toggle signals from STM32)
    input wire stm32_new_chirp,
    input wire stm32_new_elevation,
    input wire stm32_new_azimuth,
    input wire stm32_mixers_enable,
    
    // ========== FT601 USB 3.0 INTERFACE ==========
    
    // Data bus
    inout wire [31:0] ft601_data,         // 32-bit bidirectional data bus
    output wire [3:0] ft601_be,            // Byte enable (4 lanes for 32-bit mode)
    
    // Control signals
    output wire ft601_txe_n,                // Transmit enable (active low)
    output wire ft601_rxf_n,                // Receive enable (active low)
    input wire ft601_txe,                    // Transmit FIFO empty
    input wire ft601_rxf,                    // Receive FIFO full
    output wire ft601_wr_n,                  // Write strobe (active low)
    output wire ft601_rd_n,                  // Read strobe (active low)
    output wire ft601_oe_n,                  // Output enable (active low)
    output wire ft601_siwu_n,                 // Send immediate / Wakeup
    
    // FIFO flags
    input wire [1:0] ft601_srb,              // Selected read buffer
    input wire [1:0] ft601_swb,               // Selected write buffer
    
    // Clock output (optional, FT601 only — not used for FT2232H)
    output wire ft601_clk_out,
    
    // ========== FT2232H USB 2.0 INTERFACE (USB_MODE=1) ==========
    // 8-bit bidirectional data bus (245 Synchronous FIFO mode, Channel A)
    inout wire [7:0] ft_data,            // 8-bit bidirectional data bus
    input wire ft_rxf_n,                  // RX FIFO not empty (active low)
    input wire ft_txe_n,                  // TX FIFO not full (active low)
    output wire ft_rd_n,                  // Read strobe (active low)
    output wire ft_wr_n,                  // Write strobe (active low)
    output wire ft_oe_n,                  // Output enable / bus direction
    output wire ft_siwu,                  // Send Immediate / WakeUp

    // ========== STATUS OUTPUTS ==========
    
    // Beam position tracking
    output wire [5:0] current_elevation,
    output wire [5:0] current_azimuth,
    output wire [5:0] current_chirp,
    output wire new_chirp_frame,
    
    // Doppler processing outputs (for debugging)
    output wire [31:0] dbg_doppler_data,
    output wire dbg_doppler_valid,
    output wire [4:0] dbg_doppler_bin,
    output wire [`RP_RANGE_BIN_BITS-1:0] dbg_range_bin,
    
    // System status
    output wire [3:0] system_status,

    // FPGA→STM32 GPIO outputs (DIG_5..DIG_7 on 50T board)
    // Used by STM32 outer AGC loop to read saturation state without USB polling.
    output wire gpio_dig5,          // DIG_5 (H11→PD13): AGC saturation flag (1=clipping detected)
    output wire gpio_dig6,          // DIG_6 (G12→PD14): AGC enable flag (mirrors host_agc_enable)
    output wire gpio_dig7           // DIG_7 (H12→PD15): reserved (tied low)
);

// ============================================================================
// PARAMETERS
// ============================================================================

// System configuration
parameter USE_LONG_CHIRP = 1'b1;          // Default to long chirp
parameter DOPPLER_ENABLE = 1'b1;           // Enable Doppler processing
parameter USB_ENABLE = 1'b1;               // Enable USB data transfer
parameter USB_MODE = 1;                    // 0=FT601 (32-bit, 200T), 1=FT2232H (8-bit, 50T production default)

// ============================================================================
// INTERNAL SIGNALS
// ============================================================================

// Clock and reset
wire clk_100m_buf;
wire clk_120m_dac_buf;
wire ft601_clk_buf;
wire sys_reset_n;
wire sys_reset_120m_n;  // Reset synchronized to clk_120m_dac domain
wire sys_reset_ft601_n; // Reset synchronized to ft601_clk domain

// CDC: synchronized versions of async inputs for status_reg
wire stm32_mixers_enable_100m;  // stm32_mixers_enable sync'd to clk_100m
wire ft601_txe_100m;            // ft601_txe sync'd to clk_100m

// Transmitter internal signals
wire [7:0] tx_chirp_data;
wire tx_chirp_valid;
wire tx_chirp_done;
wire tx_new_chirp_frame;        // In clk_120m_dac domain
wire tx_new_chirp_frame_sync;   // Synchronized to clk_100m domain
wire [5:0] tx_current_elevation;
wire [5:0] tx_current_azimuth;
wire [5:0] tx_current_chirp;       // In clk_120m_dac domain
wire [5:0] tx_current_chirp_sync;  // Synchronized to clk_100m domain
wire tx_current_chirp_sync_valid;

// Receiver internal signals
wire [31:0] rx_doppler_output;
wire rx_doppler_valid;
wire [4:0] rx_doppler_bin;
wire [`RP_RANGE_BIN_BITS-1:0] rx_range_bin;
wire [31:0] rx_range_profile;
wire rx_range_valid;
wire [15:0] rx_range_profile_decimated;
wire rx_range_profile_decimated_valid;
wire [15:0] rx_doppler_real;
wire [15:0] rx_doppler_imag;
wire rx_doppler_data_valid;
reg rx_detect_flag;       // Threshold detection result (was rx_cfar_detection)
reg rx_detect_valid;      // Detection valid pulse (was rx_cfar_valid)

// Frame-complete signal from Doppler processor (for CFAR)
wire rx_frame_complete;

// ADC debug tap from receiver (clk_100m domain, post-DDC)
wire [15:0] rx_dbg_adc_i;
wire [15:0] rx_dbg_adc_q;
wire        rx_dbg_adc_valid;

// AGC status from receiver (for status readback and GPIO)
wire [7:0]  rx_agc_saturation_count;
wire [7:0]  rx_agc_peak_magnitude;
wire [3:0]  rx_agc_current_gain;

// DDC overflow diagnostics (audit F-6.1) — plumbed out of receiver so the
// DDC mixer_saturation / filter_overflow ports are no longer deleted at
// the boundary. Aggregated into gpio_dig5 alongside AGC saturation.
wire        rx_ddc_overflow_any;
wire [2:0]  rx_ddc_saturation_count;
// MTI saturation count (audit F-6.3). OR'd into gpio_dig5 for MCU visibility.
wire [7:0]  rx_mti_saturation_count;
// Range-bin decimator watchdog (audit F-6.4). High = decimator stalled.
wire        rx_range_decim_watchdog;
// CIC→FIR CDC overrun sticky (audit F-1.2). High = at least one baseband
// sample has been silently dropped between the 400 MHz CIC and 100 MHz FIR.
wire        rx_ddc_cic_fir_overrun;

// Data packing for USB
wire [31:0] usb_range_profile;
wire usb_range_valid;
wire [15:0] usb_doppler_real;
wire [15:0] usb_doppler_imag;
wire usb_doppler_valid;
wire usb_detect_flag;     // (was usb_cfar_detection)
wire usb_detect_valid;    // (was usb_cfar_valid)

// System status
reg [3:0] status_reg;

// USB host command outputs (Gap 4: USB Read Path)
// These are in the ft601_clk domain; CDC'd to clk_100m below
wire [31:0] usb_cmd_data;
wire        usb_cmd_valid;     // 1-cycle pulse in ft601_clk domain
wire [7:0]  usb_cmd_opcode;
wire [7:0]  usb_cmd_addr;
wire [15:0] usb_cmd_value;

// USB command decode registers (clk_100m domain, driven by CDC block below)
// Declared here (before rx_inst) so Icarus Verilog can resolve forward refs.
reg [1:0]  host_radar_mode;
reg        host_trigger_pulse;
reg [15:0] host_detect_threshold;  // (was host_cfar_threshold)
reg [5:0]  host_stream_control;

// Fix 3: Digital gain control register
// [3]=direction: 0=amplify, 1=attenuate. [2:0]=shift amount 0..7.
// Default 0x00 = pass-through (no gain change).
reg [3:0]  host_gain_shift;

// Gap 2: Host-configurable chirp timing registers
// These override the compile-time defaults in radar_mode_controller when
// written via USB command. Defaults match the parameter values in
// radar_mode_controller.v so behavior is unchanged until the host writes them.
reg [15:0] host_long_chirp_cycles;    // Opcode 0x10 (default 3000)
reg [15:0] host_long_listen_cycles;   // Opcode 0x11 (default 13700)
reg [15:0] host_guard_cycles;         // Opcode 0x12 (default 17540)
reg [15:0] host_short_chirp_cycles;   // Opcode 0x13 (default 50)
reg [15:0] host_short_listen_cycles;  // Opcode 0x14 (default 17450)
reg [5:0]  host_chirps_per_elev;      // Opcode 0x15 (default 32)
reg        host_status_request;       // Opcode 0xFF (self-clearing pulse)

// Fix 4: Doppler/chirps mismatch protection
// DOPPLER_FRAME_CHIRPS is the fixed chirp count expected by the staggered-PRI
// Doppler path (16 long + 16 short). If host sets chirps_per_elev to a
// different value, Doppler accumulation is corrupted. Clamp at command decode
// and flag the mismatch so the host knows.
localparam DOPPLER_FRAME_CHIRPS = 32; // Total chirps per Doppler frame
reg        chirps_mismatch_error;     // Set if host tried to set chirps != FFT size

// Range-mode register (opcode 0x20)
// Controls chirp type selection in the mode controller:
//   2'b00 = 3 km mode (all short chirps — long blind zone > max range)
//   2'b01 = Long-range (dual chirp: first half long, second half short)
//   2'b10 = Reserved
//   2'b11 = Reserved
reg [1:0]  host_range_mode;

// CFAR configuration registers (host-configurable via USB)
reg [3:0]  host_cfar_guard;       // Opcode 0x21: guard cells per side (0..8)
reg [4:0]  host_cfar_train;       // Opcode 0x22: training cells per side (1..16)
reg [7:0]  host_cfar_alpha;       // Opcode 0x23: threshold multiplier (Q4.4)
reg [1:0]  host_cfar_mode;        // Opcode 0x24: 00=CA, 01=GO, 10=SO
reg        host_cfar_enable;      // Opcode 0x25: 1=CFAR, 0=simple threshold

// Ground clutter removal registers (host-configurable via USB)
reg        host_mti_enable;       // Opcode 0x26: 1=MTI active, 0=pass-through
reg [2:0]  host_dc_notch_width;   // Opcode 0x27: DC notch ±width bins (0=off, 1..7)

// AGC configuration registers (host-configurable via USB, opcodes 0x28-0x2C)
reg        host_agc_enable;       // Opcode 0x28: 0=manual gain, 1=auto AGC
reg [7:0]  host_agc_target;       // Opcode 0x29: target peak magnitude (default 200)
reg [3:0]  host_agc_attack;       // Opcode 0x2A: gain-down step on clipping (default 1)
reg [3:0]  host_agc_decay;        // Opcode 0x2B: gain-up step when weak (default 1)
reg [3:0]  host_agc_holdoff;      // Opcode 0x2C: frames to wait before gain-up (default 4)

// Board bring-up self-test registers (opcode 0x30 trigger, 0x31 readback)
reg        host_self_test_trigger;  // Opcode 0x30: self-clearing pulse
wire       self_test_busy;
wire       self_test_result_valid;
wire [4:0] self_test_result_flags;  // Per-test PASS(1)/FAIL(0)
wire [7:0] self_test_result_detail; // Diagnostic detail byte
// Self-test latched results (hold until next trigger)
reg  [4:0] self_test_flags_latched;
reg  [7:0] self_test_detail_latched;
// Self-test ADC capture wires
wire       self_test_capture_active;
wire [15:0] self_test_capture_data;
wire       self_test_capture_valid;

// ============================================================================
// CLOCK BUFFERING
// ============================================================================

`ifdef SIMULATION
// In simulation (iverilog), BUFG is not available — pass-through assigns
assign clk_100m_buf     = clk_100m;
assign clk_120m_dac_buf = clk_120m_dac;
assign ft601_clk_buf    = ft601_clk_in;
`else
BUFG bufg_100m (
    .I(clk_100m),
    .O(clk_100m_buf)
);

BUFG bufg_120m (
    .I(clk_120m_dac),
    .O(clk_120m_dac_buf)
);

BUFG bufg_ft601 (
    .I(ft601_clk_in),
    .O(ft601_clk_buf)
);
`endif

// Reset synchronization (clk_100m domain)
(* ASYNC_REG = "TRUE" *) reg [1:0] reset_sync;
always @(posedge clk_100m_buf or negedge reset_n) begin
    if (!reset_n) begin
        reset_sync <= 2'b00;
    end else begin
        reset_sync <= {reset_sync[0], 1'b1};
    end
end
assign sys_reset_n = reset_sync[1];

// Reset synchronization (clk_120m_dac domain)
// Ensures reset deassertion is synchronous to the DAC clock,
// preventing recovery/removal timing violations on 120 MHz FFs.
(* ASYNC_REG = "TRUE" *) reg [1:0] reset_sync_120m;
always @(posedge clk_120m_dac_buf or negedge reset_n) begin
    if (!reset_n) begin
        reset_sync_120m <= 2'b00;
    end else begin
        reset_sync_120m <= {reset_sync_120m[0], 1'b1};
    end
end
assign sys_reset_120m_n = reset_sync_120m[1];

// Reset synchronization (ft601_clk domain)
// FT601 has its own asynchronous clock from the USB controller.
// All FT601-domain registers need a properly synchronized reset.
(* ASYNC_REG = "TRUE" *) reg [2:0] reset_sync_ft601;  // 3-stage for better MTBF
always @(posedge ft601_clk_buf or negedge reset_n) begin
    if (!reset_n) begin
        reset_sync_ft601 <= 3'b000;
    end else begin
        reset_sync_ft601 <= {reset_sync_ft601[1:0], 1'b1};
    end
end
assign sys_reset_ft601_n = reset_sync_ft601[2];

// CDC synchronizers for status_reg inputs (async -> clk_100m)
// stm32_mixers_enable is an async GPIO; ft601_txe is on ft601_clk domain
cdc_single_bit #(.STAGES(2)) cdc_mixers_en_status (
    .src_clk(clk_100m_buf),           // Pseudo-source for async GPIO
    .dst_clk(clk_100m_buf),
    .reset_n(sys_reset_n),
    .src_signal(stm32_mixers_enable),
    .dst_signal(stm32_mixers_enable_100m)
);

cdc_single_bit #(.STAGES(2)) cdc_ft601_txe_status (
    .src_clk(ft601_clk_buf),
    .dst_clk(clk_100m_buf),
    .reset_n(sys_reset_n),
    .src_signal(ft601_txe),
    .dst_signal(ft601_txe_100m)
);

// ============================================================================
// CLOCK DOMAIN CROSSING: TRANSMITTER (120 MHz) -> SYSTEM (100 MHz)
// ============================================================================

// CDC for chirp_counter: 6-bit multi-bit Gray-code synchronizer
// Source domain is clk_120m_dac, so reset must be synchronized to that domain.
// The cdc_adc_to_processing module uses synchronous reset internally, so
// using sys_reset_120m_n (120m-synchronized) is correct for the source side.
// The destination side will sample it synchronously on dst_clk, which at worst
// delays reset deassertion by 1-2 cycles — acceptable for CDC reset.
cdc_adc_to_processing #(
    .WIDTH(6),
    .STAGES(3)
) cdc_chirp_counter (
    .src_clk(clk_120m_dac_buf),
    .dst_clk(clk_100m_buf),
    .src_reset_n(sys_reset_120m_n),
    .dst_reset_n(sys_reset_n),
    .src_data(tx_current_chirp),
    .src_valid(1'b1),           // Always valid — counter updates continuously
    .dst_data(tx_current_chirp_sync),
    .dst_valid(tx_current_chirp_sync_valid)
);

// CDC for new_chirp_frame: toggle CDC (pulse on clk_120m -> pulse on clk_100m)
// new_chirp_frame is a 1-cycle pulse on clk_120m_dac. A level synchronizer
// at 100 MHz can miss it. Toggle CDC converts pulse -> level toggle,
// synchronizes the toggle, then detects edges to recover the pulse.
reg chirp_frame_toggle_120m;
always @(posedge clk_120m_dac_buf or negedge sys_reset_120m_n) begin
    if (!sys_reset_120m_n)
        chirp_frame_toggle_120m <= 1'b0;
    else if (tx_new_chirp_frame)
        chirp_frame_toggle_120m <= ~chirp_frame_toggle_120m;
end

wire chirp_frame_toggle_100m;
cdc_single_bit #(
    .STAGES(3)
) cdc_new_chirp_frame (
    .src_clk(clk_120m_dac_buf),
    .dst_clk(clk_100m_buf),
    .reset_n(sys_reset_n),
    .src_signal(chirp_frame_toggle_120m),
    .dst_signal(chirp_frame_toggle_100m)
);

reg chirp_frame_toggle_100m_prev;
always @(posedge clk_100m_buf or negedge sys_reset_n) begin
    if (!sys_reset_n)
        chirp_frame_toggle_100m_prev <= 1'b0;
    else
        chirp_frame_toggle_100m_prev <= chirp_frame_toggle_100m;
end
assign tx_new_chirp_frame_sync = chirp_frame_toggle_100m ^ chirp_frame_toggle_100m_prev;

// ============================================================================
// RADAR TRANSMITTER INSTANTIATION
// ============================================================================

radar_transmitter tx_inst (
    // System Clocks
    .clk_100m(clk_100m_buf),
    .clk_120m_dac(clk_120m_dac_buf),
    .reset_n(sys_reset_120m_n),    // 120 MHz-synchronized reset for DAC-domain logic
    .reset_100m_n(sys_reset_n),    // 100 MHz-synchronized reset for edge detectors/CDC
    
    // DAC Interface
    .dac_data(dac_data),
    .dac_clk(dac_clk),
    .dac_sleep(dac_sleep),
    
    // Mixer Enables
    .rx_mixer_en(rx_mixer_en),
    .tx_mixer_en(tx_mixer_en),
    
    // STM32 Control Interface
    .stm32_new_chirp(stm32_new_chirp),
    .stm32_new_elevation(stm32_new_elevation),
    .stm32_new_azimuth(stm32_new_azimuth),
    .stm32_mixers_enable(stm32_mixers_enable),
    
    // RF Switch Control
    .fpga_rf_switch(fpga_rf_switch),
    
    // ADAR1000 Control Interface
    .adar_tx_load_1(adar_tx_load_1),
    .adar_rx_load_1(adar_rx_load_1),
    .adar_tx_load_2(adar_tx_load_2),
    .adar_rx_load_2(adar_rx_load_2),
    .adar_tx_load_3(adar_tx_load_3),
    .adar_rx_load_3(adar_rx_load_3),
    .adar_tx_load_4(adar_tx_load_4),
    .adar_rx_load_4(adar_rx_load_4),
    .adar_tr_1(adar_tr_1),
    .adar_tr_2(adar_tr_2),
    .adar_tr_3(adar_tr_3),
    .adar_tr_4(adar_tr_4),
    
    // Level Shifter SPI Interface
    .stm32_sclk_3v3(stm32_sclk_3v3),
    .stm32_mosi_3v3(stm32_mosi_3v3),
    .stm32_miso_3v3(stm32_miso_3v3),
    .stm32_cs_adar1_3v3(stm32_cs_adar1_3v3),
    .stm32_cs_adar2_3v3(stm32_cs_adar2_3v3),
    .stm32_cs_adar3_3v3(stm32_cs_adar3_3v3),
    .stm32_cs_adar4_3v3(stm32_cs_adar4_3v3),
    
    .stm32_sclk_1v8(stm32_sclk_1v8),
    .stm32_mosi_1v8(stm32_mosi_1v8),
    .stm32_miso_1v8(stm32_miso_1v8),
    .stm32_cs_adar1_1v8(stm32_cs_adar1_1v8),
    .stm32_cs_adar2_1v8(stm32_cs_adar2_1v8),
    .stm32_cs_adar3_1v8(stm32_cs_adar3_1v8),
    .stm32_cs_adar4_1v8(stm32_cs_adar4_1v8),
    
    // Beam Position Tracking
    .current_elevation(tx_current_elevation),
    .current_azimuth(tx_current_azimuth),
    .current_chirp(tx_current_chirp),
    .new_chirp_frame(tx_new_chirp_frame),

    // Range mode — drives TX chirp LUT and FSM path selection
    .cfg_range_mode(host_range_mode)
);

// ============================================================================
// RADAR RECEIVER INSTANTIATION
// ============================================================================

radar_receiver_final rx_inst (
    .clk(clk_100m_buf),
    .reset_n(sys_reset_n),
    
    // Chirp counter from transmitter (CDC-synchronized from 120 MHz domain)
    .chirp_counter(tx_current_chirp_sync),
    // Frame-start pulse from transmitter (CDC-synchronized toggle→pulse)
    .tx_frame_start(tx_new_chirp_frame_sync),
    
    // ADC Physical Interface
    .adc_d_p(adc_d_p),
    .adc_d_n(adc_d_n),
    .adc_dco_p(adc_dco_p),
    .adc_dco_n(adc_dco_n),
    .adc_or_p(adc_or_p),
    .adc_or_n(adc_or_n),
    .adc_pwdn(adc_pwdn),
    
    // Doppler Outputs
    .doppler_output(rx_doppler_output),
    .doppler_valid(rx_doppler_valid),
    .doppler_bin(rx_doppler_bin),
    .range_bin(rx_range_bin),
    
    // Range-profile outputs
    .range_profile_i_out(rx_range_profile[15:0]),
    .range_profile_q_out(rx_range_profile[31:16]),
    .range_profile_valid_out(rx_range_valid),
    .decimated_range_mag_out(rx_range_profile_decimated),
    .decimated_range_valid_out(rx_range_profile_decimated_valid),
    
    .host_mode(host_radar_mode),
    .host_trigger(host_trigger_pulse),
    .host_range_mode(host_range_mode),
    // Gap 2: Host-configurable chirp timing
    .host_long_chirp_cycles(host_long_chirp_cycles),
    .host_long_listen_cycles(host_long_listen_cycles),
    .host_guard_cycles(host_guard_cycles),
    .host_short_chirp_cycles(host_short_chirp_cycles),
    .host_short_listen_cycles(host_short_listen_cycles),
    .host_chirps_per_elev(host_chirps_per_elev),
    // Fix 3: digital gain control
    .host_gain_shift(host_gain_shift),
    // AGC configuration (opcodes 0x28-0x2C)
    .host_agc_enable(host_agc_enable),
    .host_agc_target(host_agc_target),
    .host_agc_attack(host_agc_attack),
    .host_agc_decay(host_agc_decay),
    .host_agc_holdoff(host_agc_holdoff),
    // STM32 toggle signals for RX mode controller (mode 00 pass-through).
    // These are the raw GPIO inputs — the RX mode controller's edge detectors
    // (inside radar_mode_controller) handle debouncing/edge detection.
    .stm32_new_chirp_rx(stm32_new_chirp),
    .stm32_new_elevation_rx(stm32_new_elevation),
    .stm32_new_azimuth_rx(stm32_new_azimuth),
    // CFAR: Doppler frame-complete pulse
    .doppler_frame_done_out(rx_frame_complete),
    // Ground clutter removal
    .host_mti_enable(host_mti_enable),
    .host_dc_notch_width(host_dc_notch_width),
    // ADC debug tap (for self-test / bring-up)
    .dbg_adc_i(rx_dbg_adc_i),
    .dbg_adc_q(rx_dbg_adc_q),
    .dbg_adc_valid(rx_dbg_adc_valid),
    // AGC status outputs
    .agc_saturation_count(rx_agc_saturation_count),
    .agc_peak_magnitude(rx_agc_peak_magnitude),
    .agc_current_gain(rx_agc_current_gain),
    // DDC overflow diagnostics (audit F-6.1)
    .ddc_overflow_any(rx_ddc_overflow_any),
    .ddc_saturation_count(rx_ddc_saturation_count),
    // MTI saturation count (audit F-6.3)
    .mti_saturation_count_out(rx_mti_saturation_count),
    // Range-bin decimator watchdog (audit F-6.4)
    .range_decim_watchdog(rx_range_decim_watchdog),
    .ddc_cic_fir_overrun(rx_ddc_cic_fir_overrun)
);

// ============================================================================
// DOPPLER DATA DECODING
// ============================================================================

// Decode 32-bit doppler output into real and imaginary parts
// Format: {doppler_q[15:0], doppler_i[15:0]}
assign rx_doppler_real = rx_doppler_output[15:0];
assign rx_doppler_imag = rx_doppler_output[31:16];
assign rx_doppler_data_valid = rx_doppler_valid;

// ============================================================================
// DC NOTCH FILTER (post-Doppler-FFT, pre-CFAR)
// ============================================================================
// Zeros out Doppler bins within ±host_dc_notch_width of DC for BOTH
// sub-frames in the dual 16-pt FFT architecture.
// doppler_bin[4:0] = {sub_frame, bin[3:0]}:
//   Sub-frame 0: bins 0-15,  DC = bin 0,  wrap = bin 15
//   Sub-frame 1: bins 16-31, DC = bin 16, wrap = bin 31
// notch_width=1 → zero bins {0,16}. notch_width=2 → zero bins
// {0,1,15,16,17,31}. etc.
// When host_dc_notch_width=0: pass-through (no zeroing).

wire dc_notch_active;
wire [4:0] dop_bin_unsigned = rx_doppler_bin;
wire [3:0] bin_within_sf = dop_bin_unsigned[3:0];
assign dc_notch_active = (host_dc_notch_width != 3'd0) &&
                          (bin_within_sf < {1'b0, host_dc_notch_width} ||
                           bin_within_sf > (4'd15 - {1'b0, host_dc_notch_width} + 4'd1));

// Notched Doppler data: zero I/Q when in notch zone, pass through otherwise
wire [31:0] notched_doppler_data  = dc_notch_active ? 32'd0 : rx_doppler_output;
wire        notched_doppler_valid = rx_doppler_valid;
wire [4:0]  notched_doppler_bin   = rx_doppler_bin;
wire [`RP_RANGE_BIN_BITS-1:0]  notched_range_bin     = rx_range_bin;

// ============================================================================
// CFAR DETECTOR (replaces simple threshold detector)
// ============================================================================
// Cell-Averaging CFAR with CA/GO/SO modes. When cfg_cfar_enable=0,
// falls back to simple magnitude threshold (backward-compatible).
// See cfar_ca.v for architecture details.

wire cfar_detect_flag;
wire cfar_detect_valid;
wire [`RP_RANGE_BIN_BITS-1:0]  cfar_detect_range;
wire [4:0]  cfar_detect_doppler;
wire [16:0] cfar_detect_magnitude;
wire [16:0] cfar_detect_threshold;
wire [15:0] cfar_detect_count;
wire        cfar_busy_w;
wire [7:0]  cfar_status_w;

cfar_ca cfar_inst (
    .clk(clk_100m_buf),
    .reset_n(sys_reset_n),

    // Doppler processor outputs (DC-notch filtered)
    .doppler_data(notched_doppler_data),
    .doppler_valid(notched_doppler_valid),
    .doppler_bin_in(notched_doppler_bin),
    .range_bin_in(notched_range_bin),
    .frame_complete(rx_frame_complete),

    // Configuration
    .cfg_guard_cells(host_cfar_guard),
    .cfg_train_cells(host_cfar_train),
    .cfg_alpha(host_cfar_alpha),
    .cfg_cfar_mode(host_cfar_mode),
    .cfg_cfar_enable(host_cfar_enable),
    .cfg_simple_threshold(host_detect_threshold),

    // Detection outputs
    .detect_flag(cfar_detect_flag),
    .detect_valid(cfar_detect_valid),
    .detect_range(cfar_detect_range),
    .detect_doppler(cfar_detect_doppler),
    .detect_magnitude(cfar_detect_magnitude),
    .detect_threshold(cfar_detect_threshold),

    // Status
    .detect_count(cfar_detect_count),
    .cfar_busy(cfar_busy_w),
    .cfar_status(cfar_status_w)
);

// Connect CFAR outputs to existing detection signals
// (rx_detect_flag/valid are regs — drive them from CFAR combinationally)
always @(posedge clk_100m_buf or negedge sys_reset_n) begin
    if (!sys_reset_n) begin
        rx_detect_flag  <= 1'b0;
        rx_detect_valid <= 1'b0;
    end else begin
        rx_detect_flag  <= cfar_detect_flag;
        rx_detect_valid <= cfar_detect_valid;
    end
end

// ============================================================================
// BOARD BRING-UP SELF-TEST (opcode 0x30 trigger, 0x31 readback)
// ============================================================================
// Exercises key subsystems independently on first power-on.
// ADC data input is tied to real ADC data.

fpga_self_test self_test_inst (
    .clk(clk_100m_buf),
    .reset_n(sys_reset_n),
    .trigger(host_self_test_trigger),
    .busy(self_test_busy),
    .result_valid(self_test_result_valid),
    .result_flags(self_test_result_flags),
    .result_detail(self_test_result_detail),
    .adc_data_in(rx_dbg_adc_i),   // Post-DDC I channel (clk_100m, 16-bit signed)
    .adc_valid_in(rx_dbg_adc_valid), // DDC output valid (clk_100m)
    .capture_active(self_test_capture_active),
    .capture_data(self_test_capture_data),
    .capture_valid(self_test_capture_valid)
);

// Latch self-test results when valid (hold until next trigger)
always @(posedge clk_100m_buf or negedge sys_reset_n) begin
    if (!sys_reset_n) begin
        self_test_flags_latched  <= 5'b00000;
        self_test_detail_latched <= 8'd0;
    end else begin
        if (self_test_result_valid) begin
            self_test_flags_latched  <= self_test_result_flags;
            self_test_detail_latched <= self_test_result_detail;
        end
    end
end

// ============================================================================
// DATA PACKING FOR USB
// ============================================================================

// USB range profile must match the advertised 512-bin frame payload, so source it
// from the decimated range stream that feeds Doppler rather than raw MF samples.
assign usb_range_profile = {16'd0, rx_range_profile_decimated};
assign usb_range_valid = rx_range_profile_decimated_valid;

assign usb_doppler_real = rx_doppler_real;
assign usb_doppler_imag = rx_doppler_imag;
assign usb_doppler_valid = rx_doppler_valid;

assign usb_detect_flag  = rx_detect_flag;
assign usb_detect_valid = rx_detect_valid;

// ============================================================================
// USB DATA INTERFACE INSTANTIATION (parametric: FT601 or FT2232H)
// ============================================================================

generate
if (USB_MODE == 0) begin : gen_ft601
    // ---- FT601 USB 3.0 (32-bit, 200T premium board) ----
    usb_data_interface usb_inst (
        .clk(clk_100m_buf),
        .reset_n(sys_reset_n),
        .ft601_reset_n(sys_reset_ft601_n),
        
        // Radar data inputs
        .range_profile(usb_range_profile),
        .range_valid(usb_range_valid),
        .doppler_real(usb_doppler_real),
        .doppler_imag(usb_doppler_imag),
        .doppler_valid(usb_doppler_valid),
        .cfar_detection(usb_detect_flag),
        .cfar_valid(usb_detect_valid),
        
        // FT601 Interface
        .ft601_data(ft601_data),
        .ft601_be(ft601_be),
        .ft601_txe_n(ft601_txe_n),
        .ft601_rxf_n(ft601_rxf_n),
        .ft601_txe(ft601_txe),
        .ft601_rxf(ft601_rxf),
        .ft601_wr_n(ft601_wr_n),
        .ft601_rd_n(ft601_rd_n),
        .ft601_oe_n(ft601_oe_n),
        .ft601_siwu_n(ft601_siwu_n),
        .ft601_srb(ft601_srb),
        .ft601_swb(ft601_swb),
        .ft601_clk_out(ft601_clk_out),
        .ft601_clk_in(ft601_clk_buf),
        
        // Host command outputs
        .cmd_data(usb_cmd_data),
        .cmd_valid(usb_cmd_valid),
        .cmd_opcode(usb_cmd_opcode),
        .cmd_addr(usb_cmd_addr),
        .cmd_value(usb_cmd_value),

        // Stream control
        .stream_control(host_stream_control),

        // Status readback inputs
        .status_request(host_status_request),
        .status_cfar_threshold(host_detect_threshold),
        .status_stream_ctrl(host_stream_control),
        .status_radar_mode(host_radar_mode),
        .status_long_chirp(host_long_chirp_cycles),
        .status_long_listen(host_long_listen_cycles),
        .status_guard(host_guard_cycles),
        .status_short_chirp(host_short_chirp_cycles),
        .status_short_listen(host_short_listen_cycles),
        .status_chirps_per_elev(host_chirps_per_elev),
        .status_range_mode(host_range_mode),

        // Self-test status readback
        .status_self_test_flags(self_test_flags_latched),
        .status_self_test_detail(self_test_detail_latched),
        .status_self_test_busy(self_test_busy),

        // AGC status readback
        .status_agc_current_gain(rx_agc_current_gain),
        .status_agc_peak_magnitude(rx_agc_peak_magnitude),
        .status_agc_saturation_count(rx_agc_saturation_count),
        .status_agc_enable(host_agc_enable)
    );

    // FT2232H ports unused in FT601 mode — tie off
    assign ft_rd_n = 1'b1;
    assign ft_wr_n = 1'b1;
    assign ft_oe_n = 1'b1;
    assign ft_siwu = 1'b0;

end else begin : gen_ft2232h
    // ---- FT2232H USB 2.0 (8-bit, 50T production board) ----
    usb_data_interface_ft2232h usb_inst (
        .clk(clk_100m_buf),
        .reset_n(sys_reset_n),
        .ft_reset_n(sys_reset_ft601_n),  // Reuse same synchronized reset
        
        // Radar data inputs
        .range_profile(usb_range_profile),
        .range_valid(usb_range_valid),
        .doppler_real(usb_doppler_real),
        .doppler_imag(usb_doppler_imag),
        .doppler_valid(usb_doppler_valid),
        .cfar_detection(usb_detect_flag),
        .cfar_valid(usb_detect_valid),

        // Bulk frame protocol inputs
        .range_bin_in(notched_range_bin),
        .doppler_bin_in(notched_doppler_bin),
        .frame_complete(rx_frame_complete),

        // FT2232H Interface
        .ft_data(ft_data),
        .ft_rxf_n(ft_rxf_n),
        .ft_txe_n(ft_txe_n),
        .ft_rd_n(ft_rd_n),
        .ft_wr_n(ft_wr_n),
        .ft_oe_n(ft_oe_n),
        .ft_siwu(ft_siwu),
        .ft_clk(ft601_clk_buf),   // Reuse BUFG'd USB clock

        // Host command outputs
        .cmd_data(usb_cmd_data),
        .cmd_valid(usb_cmd_valid),
        .cmd_opcode(usb_cmd_opcode),
        .cmd_addr(usb_cmd_addr),
        .cmd_value(usb_cmd_value),

        // Stream control
        .stream_control(host_stream_control),

        // Status readback inputs
        .status_request(host_status_request),
        .status_cfar_threshold(host_detect_threshold),
        .status_stream_ctrl(host_stream_control),
        .status_radar_mode(host_radar_mode),
        .status_long_chirp(host_long_chirp_cycles),
        .status_long_listen(host_long_listen_cycles),
        .status_guard(host_guard_cycles),
        .status_short_chirp(host_short_chirp_cycles),
        .status_short_listen(host_short_listen_cycles),
        .status_chirps_per_elev(host_chirps_per_elev),
        .status_range_mode(host_range_mode),

        // Self-test status readback
        .status_self_test_flags(self_test_flags_latched),
        .status_self_test_detail(self_test_detail_latched),
        .status_self_test_busy(self_test_busy),

        // AGC status readback
        .status_agc_current_gain(rx_agc_current_gain),
        .status_agc_peak_magnitude(rx_agc_peak_magnitude),
        .status_agc_saturation_count(rx_agc_saturation_count),
        .status_agc_enable(host_agc_enable)
    );

    // FT601 ports unused in FT2232H mode — tie off
    assign ft601_be     = 4'b0000;
    assign ft601_txe_n  = 1'b1;
    assign ft601_rxf_n  = 1'b1;
    assign ft601_wr_n   = 1'b1;
    assign ft601_rd_n   = 1'b1;
    assign ft601_oe_n   = 1'b1;
    assign ft601_siwu_n = 1'b1;
    assign ft601_clk_out = 1'b0;
end
endgenerate

// ============================================================================
// USB COMMAND CDC: ft601_clk → clk_100m (Gap 4: USB Read Path)
// ============================================================================
// cmd_valid is a 1-cycle pulse in ft601_clk. Use toggle CDC (same pattern
// as chirp_frame_toggle_120m above) to safely transfer it to clk_100m.
// cmd_data/opcode/addr/value are held stable after cmd_valid pulses, so
// we simply sample them in clk_100m when the CDC'd pulse arrives.

// Step 1: Toggle on cmd_valid pulse (ft601_clk domain)
//
// CDC INVARIANT (audit F-1.1): usb_cmd_opcode / usb_cmd_addr / usb_cmd_value
// / usb_cmd_data MUST be driven to their final values BEFORE usb_cmd_valid
// asserts, and held stable for at least (STAGES + 1) clk_100m cycles after
// (i.e., until cmd_valid_100m has pulsed in the destination domain). These
// buses cross from ft601_clk to clk_100m as quasi-static data, NOT through
// a synchronizer — only the toggle bit above is CDC'd. If a future edit
// moves the cmd_* register write to the SAME cycle as the toggle flip, or
// drops the stability hold, the clk_100m sampler at the command decoder
// will latch metastable bits and dispatch on a garbage opcode.
// The source-side FSM in usb_data_interface_ft2232h.v / usb_data_interface.v
// currently satisfies this by assigning the cmd_* buses several cycles
// before pulsing cmd_valid and leaving them stable until the next command.
reg cmd_valid_toggle_ft601;
always @(posedge ft601_clk_buf or negedge sys_reset_ft601_n) begin
    if (!sys_reset_ft601_n)
        cmd_valid_toggle_ft601 <= 1'b0;
    else if (usb_cmd_valid)
        cmd_valid_toggle_ft601 <= ~cmd_valid_toggle_ft601;
end

// Step 2: Synchronize toggle to clk_100m domain (3-stage)
wire cmd_valid_toggle_100m;
cdc_single_bit #(
    .STAGES(3)
) cdc_cmd_valid (
    .src_clk(ft601_clk_buf),
    .dst_clk(clk_100m_buf),
    .reset_n(sys_reset_n),
    .src_signal(cmd_valid_toggle_ft601),
    .dst_signal(cmd_valid_toggle_100m)
);

// Step 3: Edge-detect toggle to recover pulse in clk_100m domain
reg cmd_valid_toggle_100m_prev;
always @(posedge clk_100m_buf or negedge sys_reset_n) begin
    if (!sys_reset_n)
        cmd_valid_toggle_100m_prev <= 1'b0;
    else
        cmd_valid_toggle_100m_prev <= cmd_valid_toggle_100m;
end
wire cmd_valid_100m = cmd_valid_toggle_100m ^ cmd_valid_toggle_100m_prev;

// Step 4: Command decode registers in clk_100m domain
// Sample cmd_data fields when CDC'd valid pulse arrives. Data is stable
// because the read FSM holds cmd_opcode/addr/value until the next command.
// NOTE: reg declarations for host_radar_mode, host_trigger_pulse,
// host_detect_threshold, host_stream_control are in INTERNAL SIGNALS section
// above (before rx_inst) to avoid Icarus Verilog forward-reference errors.

always @(posedge clk_100m_buf or negedge sys_reset_n) begin
    if (!sys_reset_n) begin
        host_radar_mode    <= 2'b01;   // Default: auto-scan
        host_trigger_pulse <= 1'b0;
        host_detect_threshold <= `RP_DEF_DETECT_THRESHOLD;
        host_stream_control <= `RP_STREAM_CTRL_DEFAULT;
        host_gain_shift     <= 4'd0;
        // Chirp timing defaults — 20km preset from radar_params.vh
        host_long_chirp_cycles   <= `RP_DEF_LONG_CHIRP_CYCLES;
        host_long_listen_cycles  <= `RP_DEF_LONG_LISTEN_CYCLES;
        host_guard_cycles        <= `RP_DEF_GUARD_CYCLES;
        host_short_chirp_cycles  <= `RP_DEF_SHORT_CHIRP_CYCLES;
        host_short_listen_cycles <= `RP_DEF_SHORT_LISTEN_CYCLES;
        host_chirps_per_elev     <= `RP_DEF_CHIRPS_PER_ELEV;
        host_status_request      <= 1'b0;
        chirps_mismatch_error    <= 1'b0;
        // Default range mode: 20km (RP_RANGE_MODE_20KM) — opcode 0x20 switches to 3km
        host_range_mode          <= `RP_RANGE_MODE_20KM;
        // CFAR defaults
        host_cfar_guard         <= `RP_DEF_CFAR_GUARD;
        host_cfar_train         <= `RP_DEF_CFAR_TRAIN;
        host_cfar_alpha         <= `RP_DEF_CFAR_ALPHA;
        host_cfar_mode          <= 2'b00;     // CA-CFAR
        host_cfar_enable        <= 1'b0;      // Disabled (simple threshold)
        // Ground clutter removal defaults (disabled — backward-compatible)
        host_mti_enable         <= 1'b0;      // MTI off
        host_dc_notch_width     <= 3'd0;      // DC notch off
        // AGC defaults (disabled — backward-compatible with manual gain)
        host_agc_enable         <= 1'b0;      // AGC off (manual gain)
        host_agc_target         <= 8'd200;    // Target peak magnitude
        host_agc_attack         <= 4'd1;      // 1-step gain-down on clipping
        host_agc_decay          <= 4'd1;      // 1-step gain-up when weak
        host_agc_holdoff        <= 4'd4;      // 4 frames before gain-up
        // Self-test defaults
        host_self_test_trigger  <= 1'b0;      // Self-test idle
    end else begin
        host_trigger_pulse <= 1'b0;    // Self-clearing pulse
        host_status_request <= 1'b0;   // Self-clearing pulse
        host_self_test_trigger <= 1'b0;  // Self-clearing pulse
        if (cmd_valid_100m) begin
            case (usb_cmd_opcode)
                8'h01: host_radar_mode     <= usb_cmd_value[1:0];
                8'h02: host_trigger_pulse  <= 1'b1;
                8'h03: host_detect_threshold <= usb_cmd_value;
                8'h04: host_stream_control <= usb_cmd_value[5:0];
                // Gap 2: chirp timing configuration
                8'h10: host_long_chirp_cycles  <= usb_cmd_value;
                8'h11: host_long_listen_cycles <= usb_cmd_value;
                8'h12: host_guard_cycles       <= usb_cmd_value;
                8'h13: host_short_chirp_cycles <= usb_cmd_value;
                8'h14: host_short_listen_cycles <= usb_cmd_value;
                8'h15: begin
                    // Fix 4: Clamp chirps_per_elev to the fixed Doppler frame size.
                    // If host requests a different value, clamp and set error flag.
                    if (usb_cmd_value[5:0] > DOPPLER_FRAME_CHIRPS[5:0]) begin
                        host_chirps_per_elev  <= DOPPLER_FRAME_CHIRPS[5:0];
                        chirps_mismatch_error <= 1'b1;
                    end else if (usb_cmd_value[5:0] == 6'd0) begin
                        host_chirps_per_elev  <= DOPPLER_FRAME_CHIRPS[5:0];
                        chirps_mismatch_error <= 1'b1;
                    end else begin
                        host_chirps_per_elev  <= usb_cmd_value[5:0];
                        // Clear error only if value matches FFT size exactly
                        chirps_mismatch_error <= (usb_cmd_value[5:0] != DOPPLER_FRAME_CHIRPS[5:0]);
                    end
                end
                8'h16: host_gain_shift         <= usb_cmd_value[3:0];  // Fix 3: digital gain
                // Opcode 0x20: atomic range-mode preset load.
                // Writes host_range_mode AND all RX timing registers in one cycle
                // so TX and RX switch coherently with no intermediate state.
                // TX timing is driven directly from cfg_range_mode inside
                // plfm_chirp_controller_enhanced — no separate TX opcode needed.
                // Encoding from radar_params.vh:
                //   RP_RANGE_MODE_3KM  (2'b00) = 3km  — medium chirp only
                //   RP_RANGE_MODE_20KM (2'b01) = 20km — long + short chirp
                8'h20: begin
                    host_range_mode <= usb_cmd_value[1:0];
                    if (usb_cmd_value[1:0] == `RP_RANGE_MODE_3KM) begin
                        // 3km preset — all timings @ 100 MHz from radar_params.vh
                        host_long_chirp_cycles   <= `RP_DEF_MED_CHIRP_CYCLES;
                        host_long_listen_cycles  <= `RP_DEF_MED_LISTEN_CYCLES;
                        host_guard_cycles        <= 16'd0;   // no guard in 3km mode
                        host_short_chirp_cycles  <= 16'd0;   // short chirp suppressed
                        host_short_listen_cycles <= 16'd0;
                        host_chirps_per_elev     <= `RP_DEF_CHIRPS_PER_ELEV;
                    end else begin
                        // 20km preset — all timings @ 100 MHz from radar_params.vh
                        host_long_chirp_cycles   <= `RP_DEF_LONG_CHIRP_CYCLES;
                        host_long_listen_cycles  <= `RP_DEF_LONG_LISTEN_CYCLES;
                        host_guard_cycles        <= `RP_DEF_GUARD_CYCLES;
                        host_short_chirp_cycles  <= `RP_DEF_SHORT_CHIRP_CYCLES;
                        host_short_listen_cycles <= `RP_DEF_SHORT_LISTEN_CYCLES;
                        host_chirps_per_elev     <= `RP_DEF_CHIRPS_PER_ELEV;
                    end
                end
                // CFAR configuration opcodes
                8'h21: host_cfar_guard         <= usb_cmd_value[3:0];
                8'h22: host_cfar_train         <= usb_cmd_value[4:0];
                8'h23: host_cfar_alpha         <= usb_cmd_value[7:0];
                8'h24: host_cfar_mode          <= usb_cmd_value[1:0];
                8'h25: host_cfar_enable        <= usb_cmd_value[0];
                // Ground clutter removal opcodes
                8'h26: host_mti_enable         <= usb_cmd_value[0];
                8'h27: host_dc_notch_width     <= usb_cmd_value[2:0];
                // AGC configuration opcodes
                8'h28: host_agc_enable         <= usb_cmd_value[0];
                8'h29: host_agc_target         <= usb_cmd_value[7:0];
                8'h2A: host_agc_attack         <= usb_cmd_value[3:0];
                8'h2B: host_agc_decay          <= usb_cmd_value[3:0];
                8'h2C: host_agc_holdoff        <= usb_cmd_value[3:0];
                // Board bring-up self-test opcodes
                8'h30: host_self_test_trigger  <= 1'b1;  // Trigger self-test
                8'h31: host_status_request     <= 1'b1;  // Self-test readback (status alias)
                // 0x31: readback handled via status mechanism (latched results)
                8'hFF: host_status_request     <= 1'b1;  // Gap 2: status readback
                default: ;
            endcase
        end
    end
end

// ============================================================================
// OUTPUT ASSIGNMENTS
// ============================================================================

assign current_elevation = tx_current_elevation;
assign current_azimuth = tx_current_azimuth;
assign current_chirp = tx_current_chirp_sync;        // Use CDC-synchronized version
assign new_chirp_frame = tx_new_chirp_frame_sync;    // Use CDC-synchronized version

assign dbg_doppler_data = rx_doppler_output;
assign dbg_doppler_valid = rx_doppler_valid;
assign dbg_doppler_bin = rx_doppler_bin;
assign dbg_range_bin = rx_range_bin;

// ============================================================================
// SYSTEM STATUS MONITORING
// ============================================================================

always @(posedge clk_100m_buf or negedge sys_reset_n) begin
    if (!sys_reset_n) begin
        status_reg <= 4'b0000;
    end else begin
        status_reg[0] <= stm32_mixers_enable_100m;  // Mixers enabled (CDC sync'd)
        status_reg[1] <= ft601_txe_100m;             // USB TX ready (CDC sync'd)
        status_reg[2] <= rx_doppler_valid;          // Data valid
        status_reg[3] <= tx_new_chirp_frame_sync;   // New chirp frame (CDC-sync'd)
    end
end

assign system_status = status_reg;

// ============================================================================
// FPGA→STM32 GPIO OUTPUTS (DIG_5, DIG_6, DIG_7)
// ============================================================================
// DIG_5: AGC saturation flag — high when per-frame saturation_count > 0.
//        STM32 reads PD13 to detect clipping and adjust ADAR1000 VGA gain.
// DIG_6: AGC enable flag — mirrors host_agc_enable so STM32 outer-loop AGC
//        tracks the FPGA register as single source of truth.
// DIG_7: Reserved (tied low for future use).
// gpio_dig5: "signal-chain clipped" — asserts on AGC saturation, DDC mixer/FIR
// overflow, or MTI 2-pulse saturation. Audit F-6.1/F-6.3: these were all
// previously invisible to the MCU.
assign gpio_dig5 = (rx_agc_saturation_count != 8'd0)
                 | rx_ddc_overflow_any
                 | (rx_ddc_saturation_count != 3'd0)
                 | (rx_mti_saturation_count != 8'd0)
                 | rx_range_decim_watchdog    // audit F-6.4
                 | rx_ddc_cic_fir_overrun;    // audit F-1.2
assign gpio_dig6 = host_agc_enable;
assign gpio_dig7 = 1'b0;

// ============================================================================
// DEBUG AND VERIFICATION
// ============================================================================

`ifdef SIMULATION
// Simulation-only debug monitoring
reg [31:0] debug_cycle_counter;
reg [31:0] data_packet_counter;

always @(posedge clk_100m_buf) begin
    debug_cycle_counter <= debug_cycle_counter + 1;
    
    if (tx_new_chirp_frame_sync) begin
        $display("[TOP] New chirp frame started at cycle %0d", debug_cycle_counter);
    end
    
    if (rx_doppler_valid) begin
        data_packet_counter <= data_packet_counter + 1;
        if (data_packet_counter < 10) begin
            $display("[TOP] Doppler data[%0d]: bin=%0d, range=%0d, I=%0d, Q=%0d",
                     data_packet_counter, rx_doppler_bin, rx_range_bin,
                     rx_doppler_real, rx_doppler_imag);
        end
    end
    
    if (data_packet_counter == 100) begin
        $display("[TOP] First 100 doppler packets processed");
    end
end
`endif

endmodule
