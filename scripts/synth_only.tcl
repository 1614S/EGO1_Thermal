#=======================================================================
# synth_only.tcl —— 只跑综合（快速验证 RTL）
#   vivado -mode batch -source scripts\synth_only.tcl
#=======================================================================

proc find_project {script_path} {
    set dir [file dirname [file normalize $script_path]]
    set cands [list [file normalize [file join $dir ..]] [pwd]]
    foreach base $cands {
        foreach rel [list Thermal.xpr prj/Thermal.xpr] {
            set x [file normalize [file join $base $rel]]
            if {[file exists $x]} { return $x }
        }
    }
    return ""
}

set xpr [find_project [info script]]
if {$xpr eq ""} { error "找不到 Thermal.xpr" }
puts "INFO: project = $xpr"
open_project $xpr
set root [file dirname $xpr]

reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1

set prog [get_property PROGRESS [get_runs synth_1]]
puts "INFO: synth progress=$prog"
if {$prog ne "100%"} {
    error "综合失败，见 $root/Thermal.runs/synth_1/runme.log"
}

open_run synth_1
file mkdir [file join $root results]
report_utilization    -file [file join $root results util_synth.rpt]
report_timing_summary -file [file join $root results timing_synth.rpt]
close_project
puts "INFO: done"
