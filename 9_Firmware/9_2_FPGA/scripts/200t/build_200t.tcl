################################################################################
# build_200t.tcl — XC7A200T Dev Board Build
# AERIS-10 Build for the 200T development board (FBG484)
#
# Target: XC7A200T-2FBG484I
# Design: radar_system_top
#
# Usage:
#   cd 9_Firmware/9_2_FPGA
#   vivado -mode batch -source scripts/200t/build_200t.tcl \
#     -log build/build.log -journal build/build.jou
################################################################################

# ==============================================================================
# 0. Configuration
# ==============================================================================

set project_name    "aeris10_radar"
set script_dir      [file dirname [file normalize [info script]]]
set project_root    [file normalize [file join $script_dir "../.."]]
set project_dir     [file join $project_root "build"]
set rtl_dir         $project_root
set top_module      "radar_system_top"
set fpga_part       "xc7a200tfbg484-2"
set report_dir      "${project_dir}/reports_build21"
set sim_dir         "${project_dir}/sim"
set bitstream_dir   "${project_dir}/bitstream"
set build_tag       "build21"

file mkdir $report_dir
file mkdir $sim_dir
file mkdir $bitstream_dir

# Record start time
set build_start [clock seconds]
set build_timestamp [clock format $build_start -format {%Y-%m-%d %H:%M:%S}]

puts "================================================================"
puts "  AERIS-10 Build 21: FFT Optimizations + E2E RTL Fixes"
puts "  Target:    $fpga_part"
puts "  Top:       $top_module"
puts "  Reports:   $report_dir"
puts "  Started:   $build_timestamp"
puts "================================================================"

# ==============================================================================
# 1. Project Creation + Source Files
# ==============================================================================

create_project $project_name $project_dir -part $fpga_part -force
set_property target_language Verilog [current_project]

# --- Add RTL sources ---
set rtl_files [list \
    "${rtl_dir}/adc_clk_mmcm.v" \
    "${rtl_dir}/ad9484_interface_400m.v" \
    "${rtl_dir}/cdc_modules.v" \
    "${rtl_dir}/chirp_memory_loader_param.v" \
    "${rtl_dir}/cic_decimator_4x_enhanced.v" \
    "${rtl_dir}/dac_interface_single.v" \
    "${rtl_dir}/ddc_400m.v" \
    "${rtl_dir}/ddc_input_interface.v" \
    "${rtl_dir}/doppler_processor.v" \
    "${rtl_dir}/edge_detector.v" \
    "${rtl_dir}/fir_lowpass.v" \
    "${rtl_dir}/frequency_matched_filter.v" \
    "${rtl_dir}/latency_buffer.v" \
    "${rtl_dir}/matched_filter_multi_segment.v" \
    "${rtl_dir}/matched_filter_processing_chain.v" \
    "${rtl_dir}/nco_400m_enhanced.v" \
    "${rtl_dir}/plfm_chirp_controller.v" \
    "${rtl_dir}/radar_mode_controller.v" \
    "${rtl_dir}/radar_receiver_final.v" \
    "${rtl_dir}/radar_system_top.v" \
    "${rtl_dir}/radar_transmitter.v" \
    "${rtl_dir}/range_bin_decimator.v" \
    "${rtl_dir}/rx_gain_control.v" \
    "${rtl_dir}/mti_canceller.v" \
    "${rtl_dir}/cfar_ca.v" \
    "${rtl_dir}/fpga_self_test.v" \
    "${rtl_dir}/usb_data_interface.v" \
    "${rtl_dir}/usb_data_interface_ft2232h.v" \
    "${rtl_dir}/xfft_16.v" \
    "${rtl_dir}/fft_engine.v" \
]

set file_count 0
foreach f $rtl_files {
    if {[file exists $f]} {
        add_files -norecurse $f
        incr file_count
    } else {
        puts "  WARNING: RTL file not found: $f"
    }
}
puts "  Added $file_count RTL files"

# Add .mem files for BRAM initialization
set mem_files [glob -nocomplain "${rtl_dir}/*.mem"]
foreach f $mem_files {
    add_files -norecurse $f
    puts "  Added MEM: [file tail $f]"
}

# Add constraints — main production XDC + MMCM supplementary XDC (FIXED)
add_files -fileset constrs_1 -norecurse [file join $project_root "constraints" "xc7a200t_fbg484.xdc"]
add_files -fileset constrs_1 -norecurse [file join $project_root "constraints" "adc_clk_mmcm.xdc"]

set_property top $top_module [current_fileset]
# FFT_XPM_BRAM: use XPM BRAM-based FFT twiddle storage (not LUTRAM)
# SUPPORT_LONG_RANGE: enable 20km data path (wider range bins, 8 segments, dual-chirp FSM)
#   Must be set on 200T builds only — 50T lacks the BRAM/LUT budget for 20km paths.
#   Mirrors radar_params.vh variant selection; mti_canceller.v and doppler_processor.v
#   gate their full range-bin widths behind this define.
set_property verilog_define {FFT_XPM_BRAM SUPPORT_LONG_RANGE} [current_fileset]
# Override USB_MODE to 0 (FT601) for 200T premium board.
# The RTL default is USB_MODE=1 (FT2232H, production 50T).
set_property generic {USB_MODE=0} [current_fileset]

# ==============================================================================
# 2. Synthesis
# ==============================================================================

puts ""
puts "================================================================"
puts "  Phase 1/5: Synthesis"
puts "================================================================"

set_property STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY rebuilt [get_runs synth_1]
set_property STEPS.SYNTH_DESIGN.ARGS.KEEP_EQUIVALENT_REGISTERS true [get_runs synth_1]

set synth_start [clock seconds]
launch_runs synth_1 -jobs 8
wait_on_run synth_1
set synth_elapsed [expr {[clock seconds] - $synth_start}]

set synth_status [get_property STATUS [get_runs synth_1]]
puts "  Synthesis status: $synth_status"
puts "  Synthesis time:   ${synth_elapsed}s ([expr {$synth_elapsed/60}]m [expr {$synth_elapsed%60}]s)"

if {[string match "*ERROR*" $synth_status] || [string match "*FAILED*" $synth_status]} {
    puts "CRITICAL: SYNTHESIS FAILED — aborting build"
    close_project
    exit 1
}

# Post-synth timing (for comparison)
open_run synth_1 -name synth_1
report_timing_summary -delay_type min_max -max_paths 10 -file "${report_dir}/01_timing_post_synth.rpt"
report_utilization -file "${report_dir}/01_utilization_post_synth.rpt"
close_design

# ==============================================================================
# 3. Implementation (opt → place → phys_opt → route → post_route_phys_opt)
# ==============================================================================

puts ""
puts "================================================================"
puts "  Phase 2/5: Implementation"
puts "================================================================"

# Aggressive directives for best timing
set_property STEPS.OPT_DESIGN.ARGS.DIRECTIVE Explore [get_runs impl_1]
set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE ExtraTimingOpt [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]

set impl_start [clock seconds]
launch_runs impl_1 -jobs 8
wait_on_run impl_1
set impl_elapsed [expr {[clock seconds] - $impl_start}]

set impl_status [get_property STATUS [get_runs impl_1]]
puts "  Implementation status: $impl_status"
puts "  Implementation time:   ${impl_elapsed}s ([expr {$impl_elapsed/60}]m [expr {$impl_elapsed%60}]s)"

if {![string match "*Complete*" $impl_status] && ![string match "*write_bitstream*" $impl_status]} {
    puts "CRITICAL: IMPLEMENTATION FAILED: $impl_status"
    close_project
    exit 1
}

# ==============================================================================
# 4. Bitstream Generation
# ==============================================================================

puts ""
puts "================================================================"
puts "  Phase 3/5: Bitstream Generation"
puts "================================================================"

set bit_start [clock seconds]
# Handle case where Vivado auto-proceeds to write_bitstream after impl
if {[catch {launch_runs impl_1 -to_step write_bitstream -jobs 8} launch_err]} {
    puts "  Note: write_bitstream may already be in progress: $launch_err"
}
wait_on_run impl_1
set bit_elapsed [expr {[clock seconds] - $bit_start}]
puts "  Bitstream time: ${bit_elapsed}s"

# Copy bitstream to known location
set bit_src "${project_dir}/aeris10_radar.runs/impl_1/${top_module}.bit"
if {[file exists $bit_src]} {
    file copy -force $bit_src "${bitstream_dir}/${top_module}_${build_tag}.bit"
    puts "  Bitstream: ${bitstream_dir}/${top_module}_${build_tag}.bit"
    puts "  Size: [file size $bit_src] bytes"
} else {
    puts "  WARNING: Bitstream file not found at $bit_src"
}

# ==============================================================================
# 5. Comprehensive Report Generation
# ==============================================================================

puts ""
puts "================================================================"
puts "  Phase 4/5: Report Generation (15-point checklist)"
puts "================================================================"

# Open the routed design for reporting
open_run impl_1 -name impl_1

# --- Checklist Item 2: Timing Summary ---
puts "  [2/15] Timing Summary..."
report_timing_summary -delay_type min_max -max_paths 100 \
    -report_unconstrained \
    -file "${report_dir}/02_timing_summary.rpt"

# --- Checklist Item 3: Clock Analysis ---
puts "  [3/15] Clock Analysis..."
report_clocks -file "${report_dir}/03_clocks.rpt"
report_clock_interaction -delay_type min_max \
    -file "${report_dir}/03_clock_interaction.rpt"
report_clock_networks -file "${report_dir}/03_clock_networks.rpt"

# --- Checklist Item 4: Utilization ---
puts "  [4/15] Utilization..."
report_utilization -file "${report_dir}/04_utilization.rpt"
report_utilization -hierarchical -file "${report_dir}/04_utilization_hierarchical.rpt"

# --- Checklist Item 5: Power ---
puts "  [5/15] Power Report..."
report_power -file "${report_dir}/05_power.rpt"

# --- Checklist Item 6: DRC ---
puts "  [6/15] DRC..."
report_drc -file "${report_dir}/06_drc.rpt"

# --- Checklist Item 7: IO and Constraints ---
puts "  [7/15] IO Report..."
report_io -file "${report_dir}/07_io.rpt"
report_timing -from [all_inputs] -to [all_outputs] -max_paths 20 \
    -file "${report_dir}/07_io_timing.rpt"

# --- Checklist Item 8: Congestion Analysis ---
puts "  [8/15] Congestion Analysis..."
report_design_analysis -congestion -file "${report_dir}/08_congestion.rpt"

# --- Checklist Item 9: Route Status ---
puts "  [9/15] Route Status..."
report_route_status -file "${report_dir}/09_route_status.rpt"

# --- Checklist Item 10: Critical Paths ---
puts "  [10/15] Critical Paths..."
report_timing -max_paths 20 -sort_by slack -nworst 5 \
    -file "${report_dir}/10_critical_paths_setup.rpt"
report_timing -delay_type min -max_paths 20 -sort_by slack -nworst 5 \
    -file "${report_dir}/10_critical_paths_hold.rpt"
report_high_fanout_nets -timing -load_type -max_nets 20 \
    -file "${report_dir}/10_high_fanout_nets.rpt"

# --- Checklist Item 11: QoR Summary ---
puts "  [11/15] QoR Summary..."
report_design_analysis -timing -file "${report_dir}/11_design_analysis_timing.rpt"
report_design_analysis -logic_level_distribution -file "${report_dir}/11_logic_level_dist.rpt"
report_methodology -file "${report_dir}/11_methodology.rpt"

# --- Checklist Item 12: CDC Analysis ---
puts "  [12/15] CDC Analysis..."
report_cdc -details -file "${report_dir}/12_cdc.rpt"

# --- Checklist Item 13: Log Scan (captured separately in build log) ---
puts "  [13/15] Log scan — see build21.log"

# --- Additional reports ---
puts "  \[extra\] Generating additional diagnostic reports..."

# report_exceptions can fail in Vivado 2025.2 — wrap in catch
if {[catch {report_exceptions -file "${report_dir}/13_exceptions.rpt"} err]} {
    puts "  WARNING: report_exceptions failed: $err"
    puts "  (Known Vivado 2025.2 issue — non-critical)"
}
if {[catch {check_timing -verbose -file "${report_dir}/13_check_timing.rpt"} err]} {
    puts "  WARNING: check_timing failed: $err"
    puts "  (Known Vivado 2025.2 issue — non-critical)"
}

# Compile configuration summary into a single text file
set summary_fh [open "${report_dir}/00_build21_summary.txt" w]
puts $summary_fh "================================================================"
puts $summary_fh "  AERIS-10 Build 21 — FFT Optimizations + E2E RTL Fixes"
puts $summary_fh "================================================================"
puts $summary_fh ""
puts $summary_fh "Build Tag:       $build_tag"
puts $summary_fh "Build Timestamp: $build_timestamp"
puts $summary_fh "FPGA Part:       $fpga_part"
puts $summary_fh "Top Module:      $top_module"
puts $summary_fh "RTL Files:       $file_count"
puts $summary_fh "Synth Status:    $synth_status"
puts $summary_fh "Synth Time:      ${synth_elapsed}s"
puts $summary_fh "Impl Status:     $impl_status"
puts $summary_fh "Impl Time:       ${impl_elapsed}s"
puts $summary_fh "Bitstream Time:  ${bit_elapsed}s"
puts $summary_fh ""

# Extract key timing numbers — use catch to handle empty STATS properties
# (Vivado 2025.2 may return empty strings after write_bitstream auto-launch)
puts $summary_fh "--- Timing ---"
if {[catch {set wns [get_property STATS.WNS [current_design]]}] || $wns eq ""} { set wns "N/A" }
if {[catch {set tns [get_property STATS.TNS [current_design]]}] || $tns eq ""} { set tns "N/A" }
if {[catch {set whs [get_property STATS.WHS [current_design]]}] || $whs eq ""} { set whs "N/A" }
if {[catch {set ths [get_property STATS.THS [current_design]]}] || $ths eq ""} { set ths "N/A" }
if {[catch {set fail_ep [get_property STATS.TPWS [current_design]]}] || $fail_ep eq ""} { set fail_ep "N/A" }
puts $summary_fh "  WNS:  $wns ns"
puts $summary_fh "  TNS:  $tns ns"
puts $summary_fh "  WHS:  $whs ns"
puts $summary_fh "  THS:  $ths ns"
puts $summary_fh ""
puts $summary_fh "  Build 20 Baseline: WNS = +0.426 ns, WHS = +0.058 ns"
puts $summary_fh "  Gap 2 Build (ref): WNS = +0.078 ns, WHS = +0.054 ns"
if {[string is double -strict $wns]} {
    puts $summary_fh "  Delta WNS vs B20: [expr {$wns - 0.426}] ns"
} else {
    puts $summary_fh "  Delta WNS vs B20: N/A (timing stats unavailable)"
}
if {[string is double -strict $whs]} {
    puts $summary_fh "  Delta WHS vs B20: [expr {$whs - 0.058}] ns"
} else {
    puts $summary_fh "  Delta WHS vs B20: N/A (timing stats unavailable)"
}
puts $summary_fh ""

# Extract utilization
puts $summary_fh "--- Utilization ---"
set lut_used   [llength [get_cells -hierarchical -filter {PRIMITIVE_TYPE =~ CLB.LUT.*}]]
set ff_used    [llength [get_cells -hierarchical -filter {PRIMITIVE_TYPE =~ CLB.FF.*}]]
set bram_used  [llength [get_cells -hierarchical -filter {PRIMITIVE_TYPE =~ BMEM.*}]]
set dsp_used   [llength [get_cells -hierarchical -filter {PRIMITIVE_TYPE =~ MULT.DSP.*}]]
puts $summary_fh "  LUTs:  $lut_used / 134600"
puts $summary_fh "  FFs:   $ff_used / 269200"
puts $summary_fh "  BRAM:  $bram_used cells"
puts $summary_fh "  DSP:   $dsp_used cells"
puts $summary_fh ""
puts $summary_fh "  Build 20 Baseline: LUTs=6092, FFs=9024, BRAM=16, DSP=140"
puts $summary_fh "  Gap 2 Build (ref): LUTs=6343, FFs=9197, BRAM=16, DSP=140"
puts $summary_fh "  Expected Build 21: DSP ~139 (−1 from barrel-shift twiddle)"
puts $summary_fh ""

# Route status
set unrouted [llength [get_nets -hierarchical -filter {ROUTE_STATUS == UNROUTED}]]
puts $summary_fh "--- Route ---"
puts $summary_fh "  Unrouted nets: $unrouted"
puts $summary_fh ""

# MMCM usage
puts $summary_fh "--- MMCM Usage (Gap 7) ---"
set mmcm_count [llength [get_cells -hierarchical -filter {PRIMITIVE_TYPE =~ CLOCK.MMCM.*}]]
puts $summary_fh "  MMCME2 used: $mmcm_count / 10"
puts $summary_fh "  Expected: 1 (adc_clk_mmcm jitter cleaner)"
puts $summary_fh ""

# Bitstream
if {[file exists $bit_src]} {
    puts $summary_fh "--- Bitstream ---"
    puts $summary_fh "  File: ${top_module}_${build_tag}.bit"
    puts $summary_fh "  Size: [file size $bit_src] bytes"
} else {
    puts $summary_fh "--- Bitstream ---"
    puts $summary_fh "  WARNING: NOT GENERATED"
}
puts $summary_fh ""

# Signoff
puts $summary_fh "--- Final Signoff ---"
set signoff_pass 1
if {![string is double -strict $wns]} {
    puts $summary_fh "  WARN: WNS = N/A (timing stats unavailable — check reports)"
} elseif {$wns < 0} {
    puts $summary_fh "  FAIL: WNS = $wns (negative slack)"
    set signoff_pass 0
} else {
    puts $summary_fh "  PASS: WNS = $wns ns (no setup violations)"
}
if {![string is double -strict $whs]} {
    puts $summary_fh "  WARN: WHS = N/A (timing stats unavailable — check reports)"
} elseif {$whs < 0} {
    puts $summary_fh "  FAIL: WHS = $whs (hold violation)"
    set signoff_pass 0
} else {
    puts $summary_fh "  PASS: WHS = $whs ns (no hold violations)"
}
if {![string is double -strict $tns]} {
    puts $summary_fh "  WARN: TNS = N/A (timing stats unavailable — check reports)"
} elseif {$tns != 0} {
    puts $summary_fh "  FAIL: TNS = $tns (total negative slack)"
    set signoff_pass 0
} else {
    puts $summary_fh "  PASS: TNS = 0 ns"
}
if {$unrouted > 0} {
    puts $summary_fh "  FAIL: $unrouted unrouted nets"
    set signoff_pass 0
} else {
    puts $summary_fh "  PASS: All nets routed"
}
if {[file exists $bit_src]} {
    puts $summary_fh "  PASS: Bitstream generated"
} else {
    puts $summary_fh "  FAIL: No bitstream"
    set signoff_pass 0
}
puts $summary_fh ""

# Timing regression check vs Build 20
if {[string is double -strict $wns] && $wns < 0.078} {
    puts $summary_fh "  *** WARNING: WNS REGRESSED below Gap 2 build (was +0.078 ns, now $wns ns) ***"
    puts $summary_fh "  *** Review critical paths — FFT opts or RTL fixes may have introduced new timing pressure ***"
}
if {[string is double -strict $whs] && $whs < 0.054} {
    puts $summary_fh "  *** WARNING: WHS REGRESSED below Gap 2 build (was +0.054 ns, now $whs ns) ***"
}

if {$signoff_pass} {
    puts $summary_fh "  *** SIGNOFF: PASS ***"
} else {
    puts $summary_fh "  *** SIGNOFF: FAIL ***"
}

close $summary_fh
puts "  Summary written to: ${report_dir}/00_build21_summary.txt"

# ==============================================================================
# 6. SDF + Timing Netlist (for post-route simulation)
# ==============================================================================

puts ""
puts "================================================================"
puts "  Phase 5/5: SDF + Timing Netlist"
puts "================================================================"

write_verilog -force -mode timesim "${sim_dir}/post_impl_timesim.v"
write_sdf -force "${sim_dir}/post_impl_timesim.sdf"

close_design
open_run synth_1 -name synth_1
write_verilog -force -mode funcsim "${sim_dir}/post_synth_funcsim.v"

# ==============================================================================
# Done
# ==============================================================================

set build_total [expr {[clock seconds] - $build_start}]
set build_end [clock format [clock seconds] -format {%Y-%m-%d %H:%M:%S}]

puts ""
puts "================================================================"
puts "  BUILD 21 COMPLETE"
puts "================================================================"
puts "  Started:    $build_timestamp"
puts "  Finished:   $build_end"
puts "  Total time: ${build_total}s ([expr {$build_total/60}]m [expr {$build_total%60}]s)"
puts "  Synth:      ${synth_elapsed}s"
puts "  Impl:       ${impl_elapsed}s"
puts "  Bitstream:  ${bit_elapsed}s"
puts "  Reports:    $report_dir"
puts "  Bitstream:  ${bitstream_dir}/${top_module}_${build_tag}.bit"
puts "  WNS: $wns ns | WHS: $whs ns | TNS: $tns ns"
puts "  Build 20 baseline: WNS +0.426 | WHS +0.058"
puts "  Gap 2 build (ref): WNS +0.078 | WHS +0.054"
if {$signoff_pass} {
    puts "  SIGNOFF: PASS"
} else {
    puts "  SIGNOFF: FAIL"
}
puts "================================================================"

close_project
puts "Done."
