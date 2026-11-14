#=======================================================================
# gui_sim_check.tcl —— 复现 GUI 的 “Run Behavioral Simulation”
#   vivado -mode batch -source scripts/gui_sim_check.tcl
#=======================================================================
set root [file normalize [file join [file dirname [info script]] ..]]
open_project [file join $root Thermal.xpr]

set_property top tb_top_smoke [get_filesets sim_1]
launch_simulation -mode behavioral
run all
close_sim
close_project
puts "INFO: gui-like sim finished"
