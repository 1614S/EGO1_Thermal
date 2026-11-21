#=======================================================================
# program.tcl —— 把比特流下载到 EGO1（XC7A35T）
#
# 用法（在仓库根目录执行）：
#   vivado -mode batch -source scripts/program.tcl
#   vivado -mode batch -source scripts/program.tcl -tclargs <xxx.bit>
#
# 不指定时按"修改时间最新"自动查找：
#   results/*.bit  →  Thermal.runs/impl_1/*.bit
#
# 注意：
#   * 比特流是易失的，断电/按 PROG 键会丢，重下即可，不用重新综合；
#   * 若 Hardware Manager 已被别的 Vivado 打开，connect_hw_server 会报
#     "cable busy"，先关掉那个窗口。
#=======================================================================
set root [file normalize [file join [file dirname [info script]] ..]]

#----------------------------------------------------------------------
# 1. 选比特流
#----------------------------------------------------------------------
if {[llength $argv] >= 1} {
    set bit [file normalize [lindex $argv 0]]
} else {
    set cands [concat \
        [glob -nocomplain [file join $root results *.bit]] \
        [glob -nocomplain [file join $root Thermal.runs impl_1 *.bit]]]
    if {[llength $cands] == 0} {
        error "找不到比特流：results/*.bit 和 Thermal.runs/impl_1/*.bit 都是空的。\
               先用 scripts/build.tcl 综合，或用 -tclargs 指定文件。"
    }
    set bit ""
    set newest 0
    foreach f $cands {
        set t [file mtime $f]
        if {$t > $newest} { set newest $t; set bit $f }
    }
}
if {![file exists $bit]} { error "比特流不存在：$bit" }
puts "INFO: bitstream = $bit"
puts "INFO: mtime     = [clock format [file mtime $bit] -format {%Y-%m-%d %H:%M:%S}]"

#----------------------------------------------------------------------
# 2. 连接 JTAG 并下载
#----------------------------------------------------------------------
open_hw_manager
if {[catch {connect_hw_server -url localhost:3121 -allow_non_jtag} msg]} {
    error "连接 hw_server 失败：$msg\n（常见原因：另一个 Vivado 已打开 Hardware Manager / 板子没上电 / USB 没插）"
}
open_hw_target
set dev [lindex [get_hw_devices] 0]
if {$dev eq ""} { error "没有识别到 FPGA 器件，检查 JTAG 连接" }
current_hw_device $dev
puts "INFO: device    = $dev"

set_property PROGRAM.FILE $bit $dev
program_hw_devices $dev
refresh_hw_device -update_hw_probes false $dev
puts "INFO: PROGRAM DONE, CONFIG_STATUS = [get_property REGISTER.CONFIG_STATUS $dev]"
puts "INFO: 判断是否成功：串口应恢复 10 帧/s（scripts\\capture_serial.ps1）"

close_hw_manager
