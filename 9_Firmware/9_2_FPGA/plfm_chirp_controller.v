`timescale 1ns / 1ps
`include "radar_params.vh"

module plfm_chirp_controller_enhanced (
    input wire clk_120m,
    input wire clk_100m,
    input wire reset_n,
    // cfg_range_mode: RP_RANGE_MODE_3KM=2'b00 (medium chirp only), RP_RANGE_MODE_20KM=2'b01 (long+short)
    input wire [1:0] cfg_range_mode,
    input wire new_chirp,
    input wire new_elevation,
    input wire new_azimuth,
    input wire mixers_enable,
    output reg [7:0] chirp_data,
    output reg chirp_valid,
	 output wire new_chirp_frame,
    output reg chirp_done,
    output reg rf_switch_ctrl,
    output wire rx_mixer_en,
    output wire tx_mixer_en,
    output wire adar_tx_load_1,
    output wire adar_rx_load_1,
    output wire adar_tx_load_2,
    output wire adar_rx_load_2,
    output wire adar_tx_load_3,
    output wire adar_rx_load_3,
    output wire adar_tx_load_4,
    output wire adar_rx_load_4,
    output reg adar_tr_1,
    output reg adar_tr_2,
    output reg adar_tr_3,
    output reg adar_tr_4,
    output reg [5:0] chirp_counter,
    output reg [5:0] elevation_counter,
    output reg [5:0] azimuth_counter
);

// Chirp parameters
parameter F_START = 30000000;       // 30 MHz
parameter F_END   = 10000000;       // 10 MHz
parameter FS      = 120000000;      // 120 MHz

// ---- Timing presets — source from radar_params.vh (all at 120 MHz) ----
localparam T1_SAMPLES_20KM    = `RP_TX_LONG_SAMPLES;      // 3600 — 30µs long chirp
localparam T1_LISTEN_20KM     = `RP_TX_LONG_LISTEN;       // 16440 — 137µs
localparam GUARD_SAMPLES      = `RP_TX_GUARD_SAMPLES;     // 21048 — guard
localparam T2_SAMPLES         = `RP_TX_SHORT_SAMPLES;     // 60 — 0.5µs short chirp
localparam T2_RADAR_LISTENING = `RP_TX_SHORT_LISTEN;      // 20940 — short listen
localparam T1_SAMPLES_3KM     = `RP_TX_MEDIUM_SAMPLES;    // 600 — 5µs medium chirp
localparam T1_LISTEN_3KM      = `RP_TX_MEDIUM_LISTEN;     // 3000 — 25µs

// Runtime MUX: 3km (RP_RANGE_MODE_3KM=2'b00) selects medium; 20km selects long
wire [15:0] T1_SAMPLES         = (cfg_range_mode == `RP_RANGE_MODE_3KM)
                                   ? T1_SAMPLES_3KM : T1_SAMPLES_20KM;
wire [15:0] T1_RADAR_LISTENING = (cfg_range_mode == `RP_RANGE_MODE_3KM)
                                   ? T1_LISTEN_3KM  : T1_LISTEN_20KM;

// Chirp and beam parameters
parameter CHIRP_MAX = 32;
parameter ELEVATION_MAX = 31;
parameter AZIMUTH_MAX = 50;

// State parameters
parameter IDLE        = 3'b000;
parameter LONG_CHIRP  = 3'b001;
parameter LONG_LISTEN = 3'b010;
parameter GUARD_TIME  = 3'b011;
parameter SHORT_CHIRP = 3'b100;
parameter SHORT_LISTEN = 3'b101;
parameter DONE        = 3'b110;

reg [2:0] current_state;
reg [2:0] next_state;

// Control registers
reg [15:0] sample_counter;

// Edge detection for input signals
wire chirp__toggling, elevation__toggling, azimuth__toggling;

// LUTs for chirp waveforms
(* ram_style = "block" *) reg [7:0] long_chirp_lut   [0:3599]; // 30µs @ 120MHz (20km)
(* ram_style = "block" *) reg [7:0] medium_chirp_lut  [0:599];  // 5µs  @ 120MHz (3km)
reg [7:0] short_chirp_lut [0:59];                               // 0.5µs inline  (20km)

// Registered BRAM read outputs (sync-only for BRAM inference)
reg [7:0] long_chirp_rd_data;
reg [7:0] medium_chirp_rd_data;

// Edge detection
assign chirp__toggling = new_chirp;
assign elevation__toggling = new_elevation;
assign azimuth__toggling = new_azimuth;
assign new_chirp_frame = (current_state == IDLE && next_state == LONG_CHIRP);

// Mixer TX/RX sequencing — mutually exclusive based on chirp FSM state.
// TX mixer active during chirp transmission, RX mixer during listen.
// Both require mixers_enable (STM32 master enable) to be high.
assign tx_mixer_en = mixers_enable && (current_state == LONG_CHIRP ||
                                       current_state == SHORT_CHIRP);
assign rx_mixer_en = mixers_enable && (current_state == LONG_LISTEN ||
                                       current_state == SHORT_LISTEN);

// ADTR1000 pull to ground tx and rx load pins if not used
assign adar_tx_load_1 = 1'b0;
assign adar_rx_load_1 = 1'b0;
assign adar_tx_load_2 = 1'b0;
assign adar_rx_load_2 = 1'b0;
assign adar_tx_load_3 = 1'b0;
assign adar_rx_load_3 = 1'b0;
assign adar_tx_load_4 = 1'b0;
assign adar_rx_load_4 = 1'b0;




// LUT Initialization — both loaded at synthesis/simulation
initial begin
    $readmemh("long_chirp_lut.mem",    long_chirp_lut);
    $readmemh("medium_chirp_lut.mem",  medium_chirp_lut);
end

// Synchronous BRAM reads (no async reset — REQP-1839/1840)
always @(posedge clk_120m) begin
    long_chirp_rd_data   <= long_chirp_lut[sample_counter];
    medium_chirp_rd_data <= medium_chirp_lut[sample_counter[9:0]];
end

// Short PLFM chirp LUT initialization (too small for BRAM, keep inline)
initial begin
    // Complete Short PLFM chirp LUT (0.5us, 30MHz to 10MHz)
    short_chirp_lut[ 0] = 8'd255; short_chirp_lut[ 1] = 8'd237; short_chirp_lut[ 2] = 8'd187; short_chirp_lut[ 3] = 8'd118; short_chirp_lut[ 4] = 8'd 49; short_chirp_lut[ 5] = 8'd  6; short_chirp_lut[ 6] = 8'd  7; short_chirp_lut[ 7] = 8'd 54; 
    short_chirp_lut[ 8] = 8'd132; short_chirp_lut[ 9] = 8'd210; short_chirp_lut[10] = 8'd253; short_chirp_lut[11] = 8'd237; short_chirp_lut[12] = 8'd167; short_chirp_lut[13] = 8'd 75; short_chirp_lut[14] = 8'd 10; short_chirp_lut[15] = 8'd 10; 
    short_chirp_lut[16] = 8'd 80; short_chirp_lut[17] = 8'd180; short_chirp_lut[18] = 8'd248; short_chirp_lut[19] = 8'd237; short_chirp_lut[20] = 8'd150; short_chirp_lut[21] = 8'd 45; short_chirp_lut[22] = 8'd  1; short_chirp_lut[23] = 8'd 54; 
    short_chirp_lut[24] = 8'd167; short_chirp_lut[25] = 8'd249; short_chirp_lut[26] = 8'd228; short_chirp_lut[27] = 8'd118; short_chirp_lut[28] = 8'd 15; short_chirp_lut[29] = 8'd 18; short_chirp_lut[30] = 8'd127; short_chirp_lut[31] = 8'd238; 
    short_chirp_lut[32] = 8'd235; short_chirp_lut[33] = 8'd118; short_chirp_lut[34] = 8'd 10; short_chirp_lut[35] = 8'd 34; short_chirp_lut[36] = 8'd167; short_chirp_lut[37] = 8'd254; short_chirp_lut[38] = 8'd187; short_chirp_lut[39] = 8'd 45; 
    short_chirp_lut[40] = 8'd  8; short_chirp_lut[41] = 8'd129; short_chirp_lut[42] = 8'd248; short_chirp_lut[43] = 8'd201; short_chirp_lut[44] = 8'd 49; short_chirp_lut[45] = 8'd 10; short_chirp_lut[46] = 8'd145; short_chirp_lut[47] = 8'd254; 
    short_chirp_lut[48] = 8'd167; short_chirp_lut[49] = 8'd 17; short_chirp_lut[50] = 8'd 46; short_chirp_lut[51] = 8'd210; short_chirp_lut[52] = 8'd235; short_chirp_lut[53] = 8'd 75; short_chirp_lut[54] = 8'd  7; short_chirp_lut[55] = 8'd155; 
    short_chirp_lut[56] = 8'd253; short_chirp_lut[57] = 8'd118; short_chirp_lut[58] = 8'd  1; short_chirp_lut[59] = 8'd129; 
end

// chirp_counter is driven solely by the clk_120m FSM always block (line ~683).
// Removed redundant clk_100m driver that caused multi-driven register
// (synthesis failure, simulation race condition).
// The FSM internally sequences through CHIRP_MAX chirps per beam position,
// so external new_chirp edge counting is unnecessary here.

// Elevation counter

always @(posedge clk_100m or negedge reset_n) begin
    if (!reset_n) begin
        elevation_counter <= 6'b1;
    end else begin 
			if (elevation__toggling) begin  
						if (elevation_counter == ELEVATION_MAX) begin
							elevation_counter <= 6'b1;
						end else begin
							elevation_counter <= elevation_counter + 6'b1;
					  end
    end
end
end


// Azimuth counter

always @(posedge clk_100m or negedge reset_n) begin
    if (!reset_n) begin
        azimuth_counter <= 6'd1;
    end else begin 
			if (azimuth__toggling) begin  
						if (azimuth_counter == AZIMUTH_MAX) begin
							azimuth_counter <= 6'd1;
						end else begin
							azimuth_counter <= azimuth_counter + 6'd1;
					  end
    end
end
end

// State register
always @(posedge clk_120m or negedge reset_n) begin
    if (!reset_n) begin
        current_state <= IDLE;
    end else begin
        current_state <= next_state;
    end
end

// Next state logic
always @(*) begin
    case (current_state)
        IDLE: begin
            if (chirp__toggling && mixers_enable)
                next_state = LONG_CHIRP;
            else
                next_state = IDLE;
        end
        
        LONG_CHIRP: begin
            if (sample_counter == T1_SAMPLES-1)
                next_state = LONG_LISTEN;
            else
                next_state = LONG_CHIRP;
        end
        
        LONG_LISTEN: begin
            if (sample_counter == T1_RADAR_LISTENING-1) begin
                if (cfg_range_mode == `RP_RANGE_MODE_3KM) begin
                    // 3km: all CHIRP_MAX chirps are medium — no short chirp phase
                    if (chirp_counter == CHIRP_MAX-1)
                        next_state = DONE;
                    else
                        next_state = LONG_CHIRP;
                end else begin
                    // 20km: first half long, then GUARD → short chirp phase
                    if (chirp_counter == (CHIRP_MAX/2)-1)
                        next_state = GUARD_TIME;
                    else
                        next_state = LONG_CHIRP;
                end
            end else begin
                next_state = LONG_LISTEN;
            end
        end
        
        GUARD_TIME: begin
            if (sample_counter == GUARD_SAMPLES-1)
                next_state = SHORT_CHIRP;
            else
                next_state = GUARD_TIME;
        end
        
        SHORT_CHIRP: begin
            if (sample_counter == T2_SAMPLES-1)
                next_state = SHORT_LISTEN;
            else
                next_state = SHORT_CHIRP;
        end
        
        SHORT_LISTEN: begin
            if (sample_counter == T2_RADAR_LISTENING-1) begin
                if (chirp_counter == CHIRP_MAX-1)
                    next_state = DONE;
                else
                    next_state = SHORT_CHIRP;
            end else begin
                next_state = SHORT_LISTEN;
            end
        end
        
        DONE: begin
            next_state = IDLE;
        end
        
        default: begin
            next_state = IDLE;
        end
    endcase
end

always @(posedge clk_120m or negedge reset_n) begin
    if (!reset_n) begin
        sample_counter <= 0;
        chirp_counter <= 0;
        chirp_valid <= 0;
        chirp_done <= 0;
        chirp_data <= 8'd128;
        rf_switch_ctrl <= 1'b0;
        adar_tr_1 <= 1'b0;
        adar_tr_2 <= 1'b0;
        adar_tr_3 <= 1'b0;
        adar_tr_4 <= 1'b0;
    end else if (mixers_enable) begin
        // Default outputs
        chirp_valid <= 0;
        chirp_done <= 0;
        rf_switch_ctrl <= 0;
        {adar_tr_1, adar_tr_2, adar_tr_3, adar_tr_4} <= 4'b0000;
        
        // Sample counter increment logic
        if (current_state == LONG_CHIRP || current_state == LONG_LISTEN || 
            current_state == GUARD_TIME || current_state == SHORT_CHIRP || 
            current_state == SHORT_LISTEN) begin
            if (sample_counter == get_max_counter(current_state, T1_SAMPLES, T1_RADAR_LISTENING) - 1) begin
                sample_counter <= 0;
                // Increment chirp counter at end of listen states
                if (current_state == LONG_LISTEN || current_state == SHORT_LISTEN) begin
                    chirp_counter <= chirp_counter + 1;
                end
            end else begin
                sample_counter <= sample_counter + 1;
            end
        end else begin
            sample_counter <= 0;
        end
        
        // State-specific outputs
        case (current_state)
            IDLE: begin
                chirp_data <= 8'd128;
            end
            
            LONG_CHIRP: begin
                rf_switch_ctrl <= 1'b1;
                {adar_tr_1, adar_tr_2, adar_tr_3, adar_tr_4} <= 4'b1111;
                if (sample_counter < T1_SAMPLES) begin
                    // 3km → medium chirp LUT; 20km → long chirp LUT
                    chirp_data  <= (cfg_range_mode == `RP_RANGE_MODE_3KM)
                                   ? medium_chirp_rd_data : long_chirp_rd_data;
                    chirp_valid <= 1'b1;
                end else begin
                    chirp_data <= 8'd128;
                end
            end
            
            LONG_LISTEN: begin
                chirp_data <= 8'd128;
				rf_switch_ctrl <= 1'b0;
            end
            
            GUARD_TIME: begin
                chirp_data <= 8'd128;
				rf_switch_ctrl <= 1'b0;
            end
            
            SHORT_CHIRP: begin
                rf_switch_ctrl <= 1'b1;
                {adar_tr_1, adar_tr_2, adar_tr_3, adar_tr_4} <= 4'b1111;
                
                // CRITICAL FIX: Generate valid signal for short chirp
                if (sample_counter < T2_SAMPLES) begin
                    chirp_data <= short_chirp_lut[sample_counter];
                    chirp_valid <= 1'b1;  // Valid during entire chirp
                end else begin
                    chirp_data <= 8'd128;
                end
            end
            
            SHORT_LISTEN: begin
                chirp_data <= 8'd128;
				rf_switch_ctrl <= 1'b0;
            end
            
            DONE: begin
                chirp_done <= 1'b1;
                chirp_data <= 8'd128;
            end
            
            default: begin
                chirp_data <= 8'd128;
            end
        endcase
    end else begin
        // Mixers disabled
        chirp_data <= 8'd128;
        chirp_valid <= 0;
        chirp_done <= 0;
        rf_switch_ctrl <= 0;
        {adar_tr_1, adar_tr_2, adar_tr_3, adar_tr_4} <= 4'b0000;
        sample_counter <= 0;
    end
end

// Helper function — timing arguments passed in because functions cannot read
// module-level wires (T1_SAMPLES / T1_RADAR_LISTENING are runtime-selected).
function [15:0] get_max_counter;
    input [2:0]  state;
    input [15:0] t1_samp;
    input [15:0] t1_listen;
    begin
        case (state)
            LONG_CHIRP:   get_max_counter = t1_samp;
            LONG_LISTEN:  get_max_counter = t1_listen;
            GUARD_TIME:   get_max_counter = GUARD_SAMPLES;
            SHORT_CHIRP:  get_max_counter = T2_SAMPLES;
            SHORT_LISTEN: get_max_counter = T2_RADAR_LISTENING;
            default:      get_max_counter = 0;
        endcase
    end
endfunction

endmodule