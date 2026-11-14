#=======================================================================
# fix_sim_define.tcl —— 给现有工程的 sim_1 补上 SIM_MODEL 宏
#   vivado -mode batch -source scripts/fix_sim_define.tcl
#=======================================================================
set root [file normalize [file join [file dirname [info script]] ..]]
set xpr  [file join $root Thermal.xpr]

open_project $xpr

set_property verilog_define {SIM_MODEL} [get_filesets sim_1]
set_property top tb_top_smoke [get_filesets sim_1]

puts "INFO: sim_1 verilog_define = [get_property verilog_define [get_filesets sim_1]]"

close_project
puts "INFO: saved"
