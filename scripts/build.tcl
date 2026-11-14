#=======================================================================
# build.tcl —— 综合 / 实现 / 比特流 + 三份报告
#   vivado -mode batch -source scripts/build.tcl
#=======================================================================
set root [file normalize [file join [file dirname [info script]] ..]]
open_project $root/prj/Thermal.xpr

reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
    error "synthesis failed"
}

launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

open_run impl_1
file mkdir $root/results
report_utilization      -file $root/results/util.rpt
report_timing_summary   -file $root/results/timing.rpt
report_power            -file $root/results/power.rpt
puts "INFO: bitstream + reports in $root/results"
