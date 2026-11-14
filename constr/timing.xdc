#=======================================================================
# timing.xdc —— 时钟、异步组、CDC 约束
#=======================================================================

# 100 MHz 输入
create_clock -period 10.000 -name clk100 [get_ports clk]

# clk_sys / clk_core / clk_ref 都由 MMCM 产生：
# 在 clk_mgr 内部的 BUFG/BUFGCE 输出上建时钟。档位可变，
# clk_core 按最高档 150 MHz（6.667 ns）约束，保证所有档位都收敛。
create_clock -period 10.000 -name clk_sys  [get_pins u_clk_mgr/u_bufg_sys/O]
create_clock -period  6.667 -name clk_core [get_pins u_clk_mgr/u_bufgce_core/O]
create_clock -period  5.000 -name clk_ref  [get_pins u_clk_mgr/u_bufg_ref/O]

# clk_core 由 MMCM 分频而来，与 clk_sys 同源但频率可变 → 按异步处理
set_clock_groups -asynchronous -group {clk_sys} -group {clk_core} -group {clk_ref}

# 唯一的跨时钟域数据：自热负载活动计数（格雷码）。
# v1 尚未把 act 接入 sys 域，接入后在此加 set_max_delay -datapath_only 2.0。

# 异步输入
set_false_path -from [get_ports rst_btn]
set_false_path -from [get_ports {btn[*]}]
set_false_path -from [get_ports {sw[*]}]
set_false_path -from [get_ports fan_tach]
set_false_path -from [get_ports uart_rx]

# 输出不需要输出延迟约束（都是慢速人机接口）
set_false_path -to [get_ports {led[*]}]
set_false_path -to [get_ports {seg[*]}]
set_false_path -to [get_ports {an[*]}]
