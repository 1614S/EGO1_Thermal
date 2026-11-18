#=======================================================================
# create_project.tcl —— 从零重建 Vivado 工程（Vivado 2026.1）
#   vivado -mode batch -source scripts/create_project.tcl
# 器件：xc7a35ticsg324-1L（工程现有器件）
#=======================================================================
set part xc7a35ticsg324-1L
set root [file normalize [file join [file dirname [info script]] ..]]

create_project -force Thermal $root/prj -part $part
set_property target_language Verilog [current_project]
set_property simulator_language Mixed [current_project]

# RTL
add_files -fileset sources_1 [glob -nocomplain $root/rtl/common/*.v]
add_files -fileset sources_1 [glob -nocomplain $root/rtl/clk/*.v]
add_files -fileset sources_1 [glob -nocomplain $root/rtl/acquire/*.v]
add_files -fileset sources_1 [glob -nocomplain $root/rtl/dsp/*.v]
add_files -fileset sources_1 [glob -nocomplain $root/rtl/ctrl/*.v]
add_files -fileset sources_1 [glob -nocomplain $root/rtl/load/*.v]
add_files -fileset sources_1 [glob -nocomplain $root/rtl/telemetry/*.v]
add_files -fileset sources_1 [glob -nocomplain $root/rtl/ui/*.v]
add_files -fileset sources_1 [glob -nocomplain $root/rtl/vplant/*.v]
add_files -fileset sources_1 [glob -nocomplain $root/rtl/top/*.v]
set_property include_dirs [list $root/rtl/pkg $root/rtl/vplant] [get_filesets sources_1]

# 约束
add_files -fileset constrs_1 $root/constr/ego1_pins.xdc
add_files -fileset constrs_1 $root/constr/timing.xdc

# 仿真
add_files -fileset sim_1 [glob -nocomplain $root/sim/tb/*.sv]
add_files -fileset sim_1 [glob -nocomplain $root/sim/models/*.v]
set_property file_type SystemVerilog [get_files [glob -nocomplain $root/sim/tb/*.sv]]
set_property include_dirs [list $root/rtl/pkg $root/sim/tb] [get_filesets sim_1]

set_property top ego1_thermal_top [current_fileset]
set_property top tb_top_smoke [get_filesets sim_1]
update_compile_order -fileset sources_1

puts "INFO: project created at $root/prj"

