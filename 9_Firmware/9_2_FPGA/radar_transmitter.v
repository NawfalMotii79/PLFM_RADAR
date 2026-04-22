`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    19:04:35 12/14/2025 
// Design Name: 
// Module Name:    radar_transmitter 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module radar_transmitter(
    // System Clocks
    input wire clk_100m,           // System clock
    input wire clk_120m_dac,       // 120MHz DAC clock
    input wire reset_n,            // Reset synchronized to clk_120m_dac
    input wire reset_100m_n,       // Reset synchronized to clk_100m (for edge detectors/CDC)
    
    // DAC Interface
    output wire [7:0] dac_data,
    output wire dac_clk,
    output wire dac_sleep,
    output wire rx_mixer_en,
    output wire tx_mixer_en,
	 
	     // STM32 Control Interface
    input wire stm32_new_chirp,
    input wire stm32_new_elevation, 
    input wire stm32_new_azimuth,
    input wire stm32_mixers_enable,
	 
	 output wire fpga_rf_switch,
	 
	     // ADAR1000 Control Interface
    output wire adar_tx_load_1,
    output wire adar_rx_load_1,
    output wire adar_tx_load_2,
    output wire adar_rx_load_2,
    output wire adar_tx_load_3,
    output wire adar_rx_load_3,
    output wire adar_tx_load_4,
    output wire adar_rx_load_4,
    output wire adar_tr_1,
    output wire adar_tr_2,
    output wire adar_tr_3,
    output wire adar_tr_4,
    
    // Level Shifter SPI Interface (STM32F7 to ADAR1000)
    input wire stm32_sclk_3v3,
    input wire stm32_mosi_3v3,
    output wire stm32_miso_3v3,
    input wire stm32_cs_adar1_3v3,
    input wire stm32_cs_adar2_3v3,
    input wire stm32_cs_adar3_3v3,
    input wire stm32_cs_adar4_3v3,
    
    output wire stm32_sclk_1v8,
    output wire stm32_mosi_1v8,
    input wire stm32_miso_1v8,
    output wire stm32_cs_adar1_1v8,
    output wire stm32_cs_adar2_1v8,
    output wire stm32_cs_adar3_1v8,
    output wire stm32_cs_adar4_1v8,
	 
			 // Beam Position Tracking
    output wire [5:0] current_elevation,
    output wire [5:0] current_azimuth,
    output wire [5:0] current_chirp,
    output wire new_chirp_frame,

    // Range mode: 2'b01=3km (medium chirp), 2'b00=20km (long+short)
    input wire [1:0] cfg_range_mode


    );
	 
// ========== SPI LEVEL SHIFTER PASSTHROUGH ==========
// FPGA bridges 3.3V STM32 SPI bus (Bank 15) to 1.8V ADAR1000 SPI bus (Bank 34).
// The FPGA I/O banks handle the actual voltage translation; these assigns
// route the signals through the fabric.
assign stm32_sclk_1v8      = stm32_sclk_3v3;
assign stm32_mosi_1v8       = stm32_mosi_3v3;
assign stm32_miso_3v3       = stm32_miso_1v8;
assign stm32_cs_adar1_1v8   = stm32_cs_adar1_3v3;
assign stm32_cs_adar2_1v8   = stm32_cs_adar2_3v3;
assign stm32_cs_adar3_1v8   = stm32_cs_adar3_3v3;
assign stm32_cs_adar4_1v8   = stm32_cs_adar4_3v3;

// Edge Detection Signals
wire new_chirp_pulse;
wire new_elevation_pulse;
wire new_azimuth_pulse;

// CDC: Synchronized versions of async STM32 GPIO inputs to clk_100m
wire stm32_new_chirp_sync;
wire stm32_new_elevation_sync;
wire stm32_new_azimuth_sync;

// CDC: Synchronized versions of signals crossing clk_100m -> clk_120m_dac
wire mixers_enable_120m;   // stm32_mixers_enable sync'd to clk_120m_dac
wire new_chirp_pulse_120m; // new_chirp_pulse (toggle CDC) in clk_120m_dac domain

// Chirp Control Signals
wire [7:0] chirp_data;
wire chirp_valid;
wire chirp_sequence_done;

// Toggle CDC for new_chirp_pulse: clk_100m -> clk_120m_dac
// Edge detector produces a 1-cycle pulse on clk_100m. A level synchronizer
// would miss it (120/100 MHz ratio). Toggle CDC converts pulse to level toggle,
// syncs the toggle, then detects edges on the destination side.
reg chirp_toggle_100m;
always @(posedge clk_100m or negedge reset_100m_n) begin
    if (!reset_100m_n)
        chirp_toggle_100m <= 1'b0;
    else if (new_chirp_pulse)
        chirp_toggle_100m <= ~chirp_toggle_100m;
end

// Sync the toggle to clk_120m_dac domain
wire chirp_toggle_120m;
cdc_single_bit #(.STAGES(3)) cdc_chirp_toggle (
    .src_clk(clk_100m),
    .dst_clk(clk_120m_dac),
    .reset_n(reset_n),
    .src_signal(chirp_toggle_100m),
    .dst_signal(chirp_toggle_120m)
);

// Detect edges on synchronized toggle to recover pulse in clk_120m domain
reg chirp_toggle_120m_prev;
always @(posedge clk_120m_dac or negedge reset_n) begin
    if (!reset_n)
        chirp_toggle_120m_prev <= 1'b0;
    else
        chirp_toggle_120m_prev <= chirp_toggle_120m;
end
assign new_chirp_pulse_120m = chirp_toggle_120m ^ chirp_toggle_120m_prev;

// Sync stm32_mixers_enable (async GPIO level) to clk_120m_dac domain
cdc_single_bit #(.STAGES(3)) cdc_mixers_en_120m (
    .src_clk(clk_100m),         // Treat as pseudo-source (GPIO is async)
    .dst_clk(clk_120m_dac),
    .reset_n(reset_n),
    .src_signal(stm32_mixers_enable),
    .dst_signal(mixers_enable_120m)
);

// CDC synchronizers: async STM32 GPIO inputs -> clk_100m domain
// These prevent metastability in the edge detectors. Without these,
// the edge detector's first FF can go metastable, and the XOR output
// can glitch, producing false chirp/elevation/azimuth pulses.
cdc_single_bit #(.STAGES(2)) cdc_stm32_chirp (
    .src_clk(clk_100m),         // Pseudo-source for async GPIO
    .dst_clk(clk_100m),
    .reset_n(reset_100m_n),
    .src_signal(stm32_new_chirp),
    .dst_signal(stm32_new_chirp_sync)
);

cdc_single_bit #(.STAGES(2)) cdc_stm32_elevation (
    .src_clk(clk_100m),
    .dst_clk(clk_100m),
    .reset_n(reset_100m_n),
    .src_signal(stm32_new_elevation),
    .dst_signal(stm32_new_elevation_sync)
);

cdc_single_bit #(.STAGES(2)) cdc_stm32_azimuth (
    .src_clk(clk_100m),
    .dst_clk(clk_100m),
    .reset_n(reset_100m_n),
    .src_signal(stm32_new_azimuth),
    .dst_signal(stm32_new_azimuth_sync)
);

// Enhanced STM32 Input Edge Detection with Debouncing
// Inputs are now CDC-synchronized (safe from metastability)
edge_detector_enhanced chirp_edge (
    .clk(clk_100m),
    .reset_n(reset_100m_n),
    .signal_in(stm32_new_chirp_sync),
    .rising_falling_edge(new_chirp_pulse)            
);

edge_detector_enhanced elevation_edge (
    .clk(clk_100m),
    .reset_n(reset_100m_n),
    .signal_in(stm32_new_elevation_sync),
    .rising_falling_edge(new_elevation_pulse)
);

edge_detector_enhanced azimuth_edge (
    .clk(clk_100m),
    .reset_n(reset_100m_n),
    .signal_in(stm32_new_azimuth_sync),
    .rising_falling_edge(new_azimuth_pulse)
);

// Enhanced PLFM Chirp Generation
plfm_chirp_controller_enhanced plfm_chirp_inst (
    .clk_120m(clk_120m_dac),
    .clk_100m(clk_100m),
    .reset_n(reset_n),
    .cfg_range_mode(cfg_range_mode),
    .new_chirp(new_chirp_pulse_120m),      // CDC-synchronized pulse in clk_120m domain
    .new_elevation(new_elevation_pulse),
    .new_azimuth(new_azimuth_pulse),
    .new_chirp_frame(new_chirp_frame),
    .mixers_enable(mixers_enable_120m),    // CDC-synchronized level in clk_120m domain
    .chirp_data(chirp_data),
    .chirp_valid(chirp_valid),
    .chirp_done(chirp_sequence_done),
    .rf_switch_ctrl(fpga_rf_switch),
    .rx_mixer_en(rx_mixer_en),
    .tx_mixer_en(tx_mixer_en),
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
    .elevation_counter(current_elevation),
    .azimuth_counter(current_azimuth),
    .chirp_counter(current_chirp)
);

// Enhanced DAC Interface
dac_interface_enhanced dac_interface_inst (
    .clk_120m(clk_120m_dac),
    .reset_n(reset_n),
    .chirp_data(chirp_data),
    .chirp_valid(chirp_valid),
    .dac_data(dac_data),
    .dac_clk(dac_clk),
    .dac_sleep(dac_sleep)
);
endmodule
