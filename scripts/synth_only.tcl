#=======================================================================
# synth_only.tcl —— 只跑综合，快速验证 RTL 与原语
#=======================================================================
set root [file normalize [file join [file dirname [info script]] ..]]
open_project [file join $root Thermal.xpr]

reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1

set prog [get_property PROGRESS [get_runs synth_1]]
set stat [get_property STATUS [get_runs synth_1]]
puts "INFO: synth progress=$prog status=$stat"

if {$prog ne "100%"} {
    puts "ERROR: 综合失败，请查看 $root/Thermal.runs/synth_1/runme.log"
    close_project
    exit 1
}

open_run synth_1
report_utilization -file $root/results/util_synth.rpt
report_timing_summary -file $root/results/timing_synth.rpt
close_project
puts "INFO: done"
