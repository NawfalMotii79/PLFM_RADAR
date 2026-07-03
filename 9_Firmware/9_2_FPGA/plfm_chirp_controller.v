`timescale 1ns / 1ps

module plfm_chirp_controller_enhanced (
    input wire clk_120m,
    input wire clk_100m,
    input wire reset_n,
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
parameter F_START = 30000000;       // 30 MHz (starting frequency)
parameter F_END = 10000000;         // 10 MHz (ending frequency)
parameter FS = 120000000;           // 120 MHz

// Timing parameters
parameter T1_SAMPLES = 3600;        // 30us at 120MHz
parameter T1_RADAR_LISTENING = 16440; //137us at 120MHz
parameter T2_SAMPLES = 60;          // 0.5us at 120MHz
parameter T2_RADAR_LISTENING = 20940; //174.5us at 120MHz
parameter GUARD_SAMPLES = 21048;    // 175.4us at 120MHz

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
(* ram_style = "block" *) reg [7:0] long_chirp_lut [0:3599];  // T1_SAMPLES-1
reg [7:0] short_chirp_lut [0:59];   // T2_SAMPLES-1

// Registered BRAM read output (sync-only for BRAM inference)
reg [7:0] long_chirp_rd_data;

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




// LUT Initialization
// Long PLFM chirp LUT loaded from .mem file for BRAM inference
initial begin
    $readmemh("long_chirp_lut.mem", long_chirp_lut);
end

// Synchronous-only BRAM read (no async reset) for BRAM inference
always @(posedge clk_120m) begin
    long_chirp_rd_data <= long_chirp_lut[sample_counter];
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
                if (chirp_counter == (CHIRP_MAX/2)-1)
                    next_state = GUARD_TIME;
                else
                    next_state = LONG_CHIRP;
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
            if (sample_counter == get_max_counter(current_state) - 1) begin
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
                
                // CRITICAL FIX: Generate valid signal
                if (sample_counter < T1_SAMPLES) begin
                    chirp_data <= long_chirp_rd_data;
                    chirp_valid <= 1'b1;  // Valid during entire chirp
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
                // RELIABILITY FIX: reset chirp_counter at burst end.
                // Without this, the counter stayed at CHIRP_MAX after DONE and
                // every subsequent burst ran 48+ long chirps before reaching
                // the (CHIRP_MAX/2)-1 GUARD transition (6-bit wrap). With the
                // MCU triggering one burst per beam block, each burst must
                // start from chirp_counter=0: 16 long + 16 short, exactly.
                chirp_counter <= 6'd0;
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

// Helper function to get max counter for each state
function [15:0] get_max_counter;
    input [2:0] state;
    begin
        case (state)
            LONG_CHIRP:   get_max_counter = T1_SAMPLES;
            LONG_LISTEN:  get_max_counter = T1_RADAR_LISTENING;
            GUARD_TIME:   get_max_counter = GUARD_SAMPLES;
            SHORT_CHIRP:  get_max_counter = T2_SAMPLES;
            SHORT_LISTEN: get_max_counter = T2_RADAR_LISTENING;
            default:      get_max_counter = 0;
        endcase
    end
endfunction

endmodule