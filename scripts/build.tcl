#=======================================================================
# build.tcl —— 综合 / 实现 / 比特流 + 报告
#
#   vivado -mode batch -source scripts\build.tcl
#   推荐：scripts\build.bat（会先 cd 到工程根目录）
#
# 工程定位：不依赖当前工作目录。从脚本自身位置向上找，
#           根目录 Thermal.xpr 或 prj\Thermal.xpr 都能用。
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
if {$xpr eq ""} {
    error "找不到 Thermal.xpr（已在脚本上级目录和当前目录的 prj 下查找）"
}
puts "INFO: project = $xpr"
open_project $xpr

# 支持：vivado -mode batch -source scripts/build.tcl -tclargs check
if {[lindex $argv 0] eq "check"} {
    puts "INFO: check ok, files=[llength [get_files]]"
    close_project
    exit 0
}

set root [file dirname $xpr]

reset_run synth_1
launch_runs synth_1 -jobs 8
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
    error "synthesis failed, see $root/Thermal.runs/synth_1/runme.log"
}

launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] != "100%"} {
    error "implementation failed, see $root/Thermal.runs/impl_1/runme.log"
}

open_run impl_1
file mkdir [file join $root results]
report_utilization    -file [file join $root results util.rpt]
report_timing_summary -file [file join $root results timing.rpt]
report_power          -file [file join $root results power.rpt]
puts "INFO: bitstream + reports in [file join $root results]"
close_project
