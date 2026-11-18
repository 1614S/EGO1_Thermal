#=======================================================================
# add_to_project.tcl —— 把骨架源码加入现有工程 Thermal.xpr
#   vivado -mode batch -source scripts/add_to_project.tcl
#
# 说明：
#   * 工程里已存在的同名文件会先移除，可重复执行；
#   * 结尾必须 close_project，否则 batch 模式不会把改动写回 .xpr。
#=======================================================================
set root [file normalize [file join [file dirname [info script]] ..]]
set xpr  [file join $root Thermal.xpr]

if {![file exists $xpr]} {
    error "找不到 $xpr"
}

open_project $xpr

# ---- 清理旧条目（可重复执行） ----
set old [get_files -quiet]
foreach f $old {
    set fn [file normalize $f]
    if {[string match "$root/rtl/*" $fn] ||
        [string match "$root/sim/*" $fn] ||
        [string match "$root/constr/*" $fn]} {
        remove_files -quiet $f
    }
}

# ---- RTL ----
set rtl_files {}
foreach d {common clk acquire dsp ctrl load telemetry ui vplant top} {
    set g [glob -nocomplain $root/rtl/$d/*.v]
    if {[llength $g] > 0} {
        add_files -fileset sources_1 -norecurse $g
        set rtl_files [concat $rtl_files $g]
    }
}
# ---- 头文件（.vh 必须显式加入，否则 Vivado 会另存一份 imports 副本）----
set hdr_files [glob -nocomplain $root/rtl/*/*.vh]
if {[llength $hdr_files] > 0} {
    add_files -fileset sources_1 -norecurse $hdr_files
    set_property file_type "Verilog Header" [get_files $hdr_files]
}
set_property include_dirs [list $root/rtl/pkg $root/rtl/vplant] [get_filesets sources_1]

# ---- 约束 ----
add_files -fileset constrs_1 -norecurse [list $root/constr/ego1_pins.xdc $root/constr/timing.xdc]

# ---- 仿真 ----
set tb_files [glob -nocomplain $root/sim/tb/*.sv]
set md_files [glob -nocomplain $root/sim/models/*.v]
if {[llength $tb_files] > 0} { add_files -fileset sim_1 -norecurse $tb_files }
if {[llength $md_files] > 0} { add_files -fileset sim_1 -norecurse $md_files }
if {[llength $tb_files] > 0} {
    set_property file_type SystemVerilog [get_files $tb_files]
}
set_property include_dirs [list $root/rtl/pkg $root/rtl/vplant $root/sim/tb] [get_filesets sim_1]
# GUI 行为仿真必须定义 SIM_MODEL：否则例化真实 XADC/MMCM 原语，
# TB 里的 tj_c 注入点不存在（VRFC 10-2991）
set_property verilog_define {SIM_MODEL} [get_filesets sim_1]

# ---- 顶层 ----
set_property top ego1_thermal_top [get_filesets sources_1]
set_property top tb_top_smoke    [get_filesets sim_1]
update_compile_order -fileset sources_1

puts "INFO: rtl=[llength $rtl_files] tb=[llength $tb_files] models=[llength $md_files]"

# 关键：写回 .xpr
close_project
puts "INFO: project saved -> $xpr"

