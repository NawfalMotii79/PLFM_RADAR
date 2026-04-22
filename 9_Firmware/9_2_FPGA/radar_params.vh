// ============================================================================
// radar_params.vh — Single Source of Truth for AERIS-10 FPGA Parameters
// ============================================================================
//
// ALL modules in the FPGA processing chain MUST `include this file instead of
// hardcoding range bins, segment counts, chirp samples, or timing values.
//
// This file uses `define macros (not localparam) so it can be included at any
// scope. Each consuming module should include this file inside its body and
// optionally alias macros to localparams for readability.
//
// BOARD VARIANTS:
//   SUPPORT_LONG_RANGE = 0  (50T, USB_MODE=1)  — 3 km mode only
//   SUPPORT_LONG_RANGE = 1  (200T, USB_MODE=0) — 3 km + 20 km modes
//
// RADAR MODES (runtime, via host_radar_mode register, opcode 0x01):
//   2'b00 = STM32 pass-through (production — STM32 controls chirp timing)
//   2'b01 = Auto-scan 3 km     (FPGA-timed, short chirps only)
//   2'b10 = Single-chirp debug (one long chirp per trigger)
//   2'b11 = Reserved / idle
//
// RANGE MODES (runtime, via host_range_mode register, opcode 0x20):
//   2'b00 = 3 km   (default — pass-through treats all chirps as short)
//   2'b01 = Long-range (pass-through: first half long, second half short)
//   2'b10 = Reserved
//   2'b11 = Reserved
//
// USAGE:
//   `include "radar_params.vh"
//   Then reference `RP_FFT_SIZE, `RP_NUM_RANGE_BINS, etc.
//
// PHYSICAL CONSTANTS (derived from hardware):
//   ADC clock:           400 MSPS
//   CIC decimation:      4x
//   Processing rate:     100 MSPS (post-DDC)
//   Range per sample:    c / (2 * 100e6) = 1.5 m
//   FFT size:            2048
//   Decimation factor:   4  (2048 FFT bins -> 512 output range bins)
//   Range per dec. bin:  1.5 m * 4 = 6.0 m
//   Max range (3 km):    512 * 6.0 = 3072 m
//   Carrier frequency:   10.5 GHz
//   IF frequency:        120 MHz
//
// CHIRP BANDWIDTH (Phase 1 target — currently 20 MHz, planned 30 MHz):
//   Range resolution:    c / (2 * BW)
//     20 MHz -> 7.5 m
//     30 MHz -> 5.0 m
//   NOTE: Range resolution is independent of range-per-bin. Resolution
//   determines the minimum separation between two targets; range-per-bin
//   determines the spatial sampling grid.
// ============================================================================

`ifndef RADAR_PARAMS_VH
`define RADAR_PARAMS_VH

// ============================================================================
// BOARD VARIANT — set at synthesis time, NOT runtime
// ============================================================================
// Default to 50T (conservative). Override in top-level or synthesis script:
//   +define+SUPPORT_LONG_RANGE
// or via Vivado: set_property verilog_define {SUPPORT_LONG_RANGE} [current_fileset]

// Note: SUPPORT_LONG_RANGE is a flag define (ifdef/ifndef), not a value.
// `ifndef SUPPORT_LONG_RANGE means 50T (no long range).
// `ifdef SUPPORT_LONG_RANGE means 200T (long range supported).

// ============================================================================
// FFT AND PROCESSING CONSTANTS (fixed, both modes)
// ============================================================================

`define RP_FFT_SIZE             2048    // Range FFT points per segment
`define RP_LOG2_FFT_SIZE        11      // log2(2048)
`define RP_OVERLAP_SAMPLES      128     // Overlap between adjacent segments
`define RP_SEGMENT_ADVANCE      1920    // FFT_SIZE - OVERLAP = 2048 - 128
`define RP_DECIMATION_FACTOR    4       // Range bin decimation (2048 -> 512)
`define RP_NUM_RANGE_BINS       512     // FFT_SIZE / DECIMATION_FACTOR
`define RP_RANGE_BIN_BITS       9       // ceil(log2(512))
`define RP_DOPPLER_FFT_SIZE     16      // Per sub-frame Doppler FFT
`define RP_CHIRPS_PER_FRAME     32      // Total chirps (16 long + 16 short)
`define RP_CHIRPS_PER_SUBFRAME  16      // Chirps per Doppler sub-frame
`define RP_NUM_DOPPLER_BINS     32      // 2 sub-frames * 16 = 32
`define RP_DATA_WIDTH           16      // ADC/processing data width

// ============================================================================
// 3 KM MODE PARAMETERS (both 50T and 200T)
// ============================================================================

`define RP_LONG_CHIRP_SAMPLES_3KM   3000    // 30 us at 100 MSPS
`define RP_LONG_SEGMENTS_3KM        2       // ceil((3000-2048)/1920) + 1 = 2
`define RP_SHORT_CHIRP_SAMPLES      50      // 0.5 us at 100 MSPS (same both modes)
`define RP_SHORT_SEGMENTS           1       // Single segment for short chirp

// Derived 3 km limits
`define RP_MAX_RANGE_3KM            3072    // 512 bins * 6 m = 3072 m

// ============================================================================
// 20 KM MODE PARAMETERS (200T only — Phase 2)
// ============================================================================

`define RP_LONG_CHIRP_SAMPLES_20KM  13700   // 137 us at 100 MSPS (= listen window)
`define RP_LONG_SEGMENTS_20KM       8       // 1 + ceil((13700-2048)/1920) = 1 + 7 = 8
`define RP_OUTPUT_RANGE_BINS_20KM   4096    // 8 segments * 512 dec. bins each

// Derived 20 km limits
`define RP_MAX_RANGE_20KM           24576   // 4096 bins * 6 m = 24576 m

// ============================================================================
// MAX VALUES (for sizing buffers — compile-time, based on board variant)
// ============================================================================

`ifdef SUPPORT_LONG_RANGE
  `define RP_MAX_SEGMENTS           8
  `define RP_MAX_OUTPUT_BINS        4096
  `define RP_MAX_CHIRP_SAMPLES      13700
`else
  `define RP_MAX_SEGMENTS           2
  `define RP_MAX_OUTPUT_BINS        512
  `define RP_MAX_CHIRP_SAMPLES      3000
`endif

// ============================================================================
// BIT WIDTHS (derived from MAX values)
// ============================================================================

// Segment index: ceil(log2(MAX_SEGMENTS))
//   50T:  log2(2) = 1 bit  (use 2 for safety)
//   200T: log2(8) = 3 bits
`ifdef SUPPORT_LONG_RANGE
  `define RP_SEGMENT_IDX_WIDTH      3
  `define RP_RANGE_BIN_WIDTH_MAX    12      // ceil(log2(4096))
  `define RP_DOPPLER_MEM_ADDR_W     17      // ceil(log2(4096*32)) = 17
  `define RP_CFAR_MAG_ADDR_W        17      // ceil(log2(4096*32)) = 17
`else
  `define RP_SEGMENT_IDX_WIDTH      2
  `define RP_RANGE_BIN_WIDTH_MAX    9       // ceil(log2(512))
  `define RP_DOPPLER_MEM_ADDR_W     14      // ceil(log2(512*32)) = 14
  `define RP_CFAR_MAG_ADDR_W        14      // ceil(log2(512*32)) = 14
`endif

// Derived depths (for memory declarations)
// Usage: reg [15:0] mem [0:`RP_DOPPLER_MEM_DEPTH-1];
`define RP_DOPPLER_MEM_DEPTH    (`RP_MAX_OUTPUT_BINS * `RP_CHIRPS_PER_FRAME)
`define RP_CFAR_MAG_DEPTH       (`RP_MAX_OUTPUT_BINS * `RP_NUM_DOPPLER_BINS)

// ============================================================================
// CHIRP TIMING — RX DOMAIN (100 MHz clock cycles)
// ============================================================================
// Reset defaults for host-configurable timing registers (opcode 0x10-0x15).
// Opcode 0x20 preset-loads the correct set atomically.
// All values are in 100 MHz cycles.  Mirror RP_TX_* values below (120 MHz).

// --- 20 km preset (RP_RANGE_MODE_20KM = 2'b01) ---
`define RP_DEF_LONG_CHIRP_CYCLES    3000    // 30 µs long chirp
`define RP_DEF_LONG_LISTEN_CYCLES   13700   // 137 µs → 20.5 km
`define RP_DEF_GUARD_CYCLES         17540   // 175.4 µs guard
`define RP_DEF_SHORT_CHIRP_CYCLES   50      // 0.5 µs short chirp
`define RP_DEF_SHORT_LISTEN_CYCLES  17450   // 174.5 µs listen
`define RP_DEF_CHIRPS_PER_ELEV      32

// --- 3 km preset (RP_RANGE_MODE_3KM = 2'b00) ---
`define RP_DEF_MED_CHIRP_CYCLES     500     // 5 µs medium chirp @ 100 MHz
`define RP_DEF_MED_LISTEN_CYCLES    2500    // 25 µs listen → 3.75 km margin

// ============================================================================
// CHIRP TIMING — TX DOMAIN (120 MHz clock cycles)
// ============================================================================
// Used by plfm_chirp_controller_enhanced.  Derived from RX values scaled by 1.2.

// --- 20 km preset ---
`define RP_TX_LONG_SAMPLES          3600    // 30 µs × 120 MHz
`define RP_TX_LONG_LISTEN           16440   // 137 µs × 120 MHz
`define RP_TX_GUARD_SAMPLES         21048   // 175.4 µs × 120 MHz
`define RP_TX_SHORT_SAMPLES         60      // 0.5 µs × 120 MHz
`define RP_TX_SHORT_LISTEN          20940   // 174.5 µs × 120 MHz

// --- 3 km preset ---
`define RP_TX_MEDIUM_SAMPLES        600     // 5 µs × 120 MHz  (TX LUT active samples)
`define RP_TX_MEDIUM_LISTEN         3000    // 25 µs × 120 MHz (listen window for 3km)

// ============================================================================
// MEDIUM CHIRP MEMORY (3 km mode — chirp_memory_loader_param.v)
// ============================================================================
`define RP_MEDIUM_CHIRP_SAMPLES_RX  500     // 5 µs at 100 MHz (RX reference IQ)
`define RP_MEDIUM_CHIRP_SAMPLES_TX  600     // 5 µs at 120 MHz (TX DAC LUT)
`define RP_MEDIUM_CHIRP_MEM_DEPTH   1024    // BRAM depth (next power of 2 ≥ 600)

// ============================================================================
// BLIND ZONE CONSTANTS (informational, for comments and GUI)
// ============================================================================
// Long chirp:   c × 30 µs / 2 = 4500 m  — 3 km target invisible
// Medium chirp: c × 5 µs / 2  =  750 m  — 3 km target visible
// Short chirp:  c × 0.5 µs / 2 =  75 m

`define RP_LONG_BLIND_ZONE_M        4500
`define RP_MEDIUM_BLIND_ZONE_M      750
`define RP_SHORT_BLIND_ZONE_M       75

// ============================================================================
// PHYSICAL CONSTANTS (integer-scaled for Verilog — use in comments/assertions)
// ============================================================================
// Range per ADC sample:     1.5 m  (stored as 15 in units of 0.1 m)
// Range per decimated bin:  6.0 m  (stored as 60 in units of 0.1 m)
// Processing rate:         100 MSPS

`define RP_RANGE_PER_SAMPLE_DM      15      // 1.5 m in decimeters
`define RP_RANGE_PER_BIN_DM         60      // 6.0 m in decimeters
`define RP_PROCESSING_RATE_MHZ      100

// ============================================================================
// AGC DEFAULTS
// ============================================================================
`define RP_DEF_AGC_TARGET           200
`define RP_DEF_AGC_ATTACK           1
`define RP_DEF_AGC_DECAY            1
`define RP_DEF_AGC_HOLDOFF          4

// ============================================================================
// CFAR DEFAULTS
// ============================================================================
`define RP_DEF_CFAR_GUARD           2
`define RP_DEF_CFAR_TRAIN           8
`define RP_DEF_CFAR_ALPHA           8'h30   // 3.0 in Q4.4
`define RP_DEF_CFAR_MODE            2'b00   // CA-CFAR

// ============================================================================
// DETECTION DEFAULTS
// ============================================================================
`define RP_DEF_DETECT_THRESHOLD     10000

// ============================================================================
// RADAR MODE ENCODING (host_radar_mode, opcode 0x01)
// ============================================================================
`define RP_MODE_STM32_PASSTHROUGH   2'b00
`define RP_MODE_AUTO_3KM            2'b01
`define RP_MODE_SINGLE_DEBUG        2'b10
`define RP_MODE_RESERVED            2'b11

// ============================================================================
// RANGE MODE ENCODING (host_range_mode / cfg_range_mode, opcode 0x20)
// ============================================================================
// 2'b00 = 3 km  — medium chirp only (no short chirp phase)
// 2'b01 = 20 km — long chirp (first half) + short chirp disambiguation
`define RP_RANGE_MODE_3KM           2'b00
`define RP_RANGE_MODE_20KM          2'b01
`define RP_RANGE_MODE_RSVD2         2'b10
`define RP_RANGE_MODE_RSVD3         2'b11

// ============================================================================
// STREAM CONTROL (host_stream_control, opcode 0x04, 6-bit)
// ============================================================================
// Bits [2:0]: Stream enable mask
//   Bit 0 = range profile stream
//   Bit 1 = doppler map stream
//   Bit 2 = cfar/detection stream
// Bits [5:3]: Stream format control
//   Bit 3 = mag_only    (0=I/Q pairs, 1=Manhattan magnitude only)
//   Bit 4 = sparse_det  (0=dense detection flags, 1=sparse detection list)
//   Bit 5 = reserved (was frame_decimate, not needed with mag-only fitting)
`define RP_STREAM_CTRL_DEFAULT      6'b001_111  // all streams, mag-only mode

`endif // RADAR_PARAMS_VH
