#=======================================================================
# ego1_pins.xdc —— EGO1 引脚约束（依据官方 EGo1.xdc / 用户手册 v2.2）
#
# 说明：
#   * 官方文件用 sys_clk_in / btn_pin 等名字，本工程用 clk / btn 等，
#     编号与封装引脚一一对应，未做任何改动。
#   * 数码管只用了 0 组（seg_cs[3:0] + seg_data_0），另一组的引脚不分配。
#=======================================================================

#-----------------------------------------------------------------------
# 时钟与复位（P17 / P15，官方 sys_clk_in / sys_rst_n）
#-----------------------------------------------------------------------
set_property -dict {PACKAGE_PIN P17 IOSTANDARD LVCMOS33} [get_ports clk]
set_property -dict {PACKAGE_PIN P15 IOSTANDARD LVCMOS33} [get_ports rst_btn]

#-----------------------------------------------------------------------
# 5 个按键：R11 R17 R15 V1 U4（按下为低，顶层已取反）
#-----------------------------------------------------------------------
set_property -dict {PACKAGE_PIN R11 IOSTANDARD LVCMOS33} [get_ports {btn[0]}]
set_property -dict {PACKAGE_PIN R17 IOSTANDARD LVCMOS33} [get_ports {btn[1]}]
set_property -dict {PACKAGE_PIN R15 IOSTANDARD LVCMOS33} [get_ports {btn[2]}]
set_property -dict {PACKAGE_PIN V1  IOSTANDARD LVCMOS33} [get_ports {btn[3]}]
set_property -dict {PACKAGE_PIN U4  IOSTANDARD LVCMOS33} [get_ports {btn[4]}]

#-----------------------------------------------------------------------
# 8 个拨码开关 sw0~sw7：P5 P4 P3 P2 R2 M4 N4 R1
#-----------------------------------------------------------------------
set_property -dict {PACKAGE_PIN P5 IOSTANDARD LVCMOS33} [get_ports {sw[0]}]
set_property -dict {PACKAGE_PIN P4 IOSTANDARD LVCMOS33} [get_ports {sw[1]}]
set_property -dict {PACKAGE_PIN P3 IOSTANDARD LVCMOS33} [get_ports {sw[2]}]
set_property -dict {PACKAGE_PIN P2 IOSTANDARD LVCMOS33} [get_ports {sw[3]}]
set_property -dict {PACKAGE_PIN R2 IOSTANDARD LVCMOS33} [get_ports {sw[4]}]
set_property -dict {PACKAGE_PIN M4 IOSTANDARD LVCMOS33} [get_ports {sw[5]}]
set_property -dict {PACKAGE_PIN N4 IOSTANDARD LVCMOS33} [get_ports {sw[6]}]
set_property -dict {PACKAGE_PIN R1 IOSTANDARD LVCMOS33} [get_ports {sw[7]}]

#-----------------------------------------------------------------------
# 16 个 LED：F6 G4 G3 J4 H4 J3 J2 K2 / K1 H6 H5 J5 K6 L1 M1 K3
#-----------------------------------------------------------------------
set_property -dict {PACKAGE_PIN F6 IOSTANDARD LVCMOS33} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN G4 IOSTANDARD LVCMOS33} [get_ports {led[1]}]
set_property -dict {PACKAGE_PIN G3 IOSTANDARD LVCMOS33} [get_ports {led[2]}]
set_property -dict {PACKAGE_PIN J4 IOSTANDARD LVCMOS33} [get_ports {led[3]}]
set_property -dict {PACKAGE_PIN H4 IOSTANDARD LVCMOS33} [get_ports {led[4]}]
set_property -dict {PACKAGE_PIN J3 IOSTANDARD LVCMOS33} [get_ports {led[5]}]
set_property -dict {PACKAGE_PIN J2 IOSTANDARD LVCMOS33} [get_ports {led[6]}]
set_property -dict {PACKAGE_PIN K2 IOSTANDARD LVCMOS33} [get_ports {led[7]}]
set_property -dict {PACKAGE_PIN K1 IOSTANDARD LVCMOS33} [get_ports {led[8]}]
set_property -dict {PACKAGE_PIN H6 IOSTANDARD LVCMOS33} [get_ports {led[9]}]
set_property -dict {PACKAGE_PIN H5 IOSTANDARD LVCMOS33} [get_ports {led[10]}]
set_property -dict {PACKAGE_PIN J5 IOSTANDARD LVCMOS33} [get_ports {led[11]}]
set_property -dict {PACKAGE_PIN K6 IOSTANDARD LVCMOS33} [get_ports {led[12]}]
set_property -dict {PACKAGE_PIN L1 IOSTANDARD LVCMOS33} [get_ports {led[13]}]
set_property -dict {PACKAGE_PIN M1 IOSTANDARD LVCMOS33} [get_ports {led[14]}]
set_property -dict {PACKAGE_PIN K3 IOSTANDARD LVCMOS33} [get_ports {led[15]}]

#-----------------------------------------------------------------------
# 数码管 0 组：段选 seg_data_0[7:0] = B4 A4 A3 B1 A1 B3 B2 D5
#             位选 seg_cs[3:0]     = G2 C2 C1 H1
# 另一组位选 seg_cs[7:4] = G1 F1 E1 G6：设计里恒灭（极性高有效 -> 输出 0）
#-----------------------------------------------------------------------
set_property -dict {PACKAGE_PIN B4 IOSTANDARD LVCMOS33} [get_ports {seg[0]}]
set_property -dict {PACKAGE_PIN A4 IOSTANDARD LVCMOS33} [get_ports {seg[1]}]
set_property -dict {PACKAGE_PIN A3 IOSTANDARD LVCMOS33} [get_ports {seg[2]}]
set_property -dict {PACKAGE_PIN B1 IOSTANDARD LVCMOS33} [get_ports {seg[3]}]
set_property -dict {PACKAGE_PIN A1 IOSTANDARD LVCMOS33} [get_ports {seg[4]}]
set_property -dict {PACKAGE_PIN B3 IOSTANDARD LVCMOS33} [get_ports {seg[5]}]
set_property -dict {PACKAGE_PIN B2 IOSTANDARD LVCMOS33} [get_ports {seg[6]}]
set_property -dict {PACKAGE_PIN D5 IOSTANDARD LVCMOS33} [get_ports {seg[7]}]

set_property -dict {PACKAGE_PIN G2 IOSTANDARD LVCMOS33} [get_ports {an[0]}]
set_property -dict {PACKAGE_PIN C2 IOSTANDARD LVCMOS33} [get_ports {an[1]}]
set_property -dict {PACKAGE_PIN C1 IOSTANDARD LVCMOS33} [get_ports {an[2]}]
set_property -dict {PACKAGE_PIN H1 IOSTANDARD LVCMOS33} [get_ports {an[3]}]
set_property -dict {PACKAGE_PIN G1 IOSTANDARD LVCMOS33} [get_ports {an[4]}]
set_property -dict {PACKAGE_PIN F1 IOSTANDARD LVCMOS33} [get_ports {an[5]}]
set_property -dict {PACKAGE_PIN E1 IOSTANDARD LVCMOS33} [get_ports {an[6]}]
set_property -dict {PACKAGE_PIN G6 IOSTANDARD LVCMOS33} [get_ports {an[7]}]

#-----------------------------------------------------------------------
# 串口：PC_Uart_rxd = N5（FPGA 输入），PC_Uart_txd = T4（FPGA 输出）
#-----------------------------------------------------------------------
set_property -dict {PACKAGE_PIN N5 IOSTANDARD LVCMOS33} [get_ports uart_rx]
set_property -dict {PACKAGE_PIN T4 IOSTANDARD LVCMOS33} [get_ports uart_tx]

#-----------------------------------------------------------------------
# Pmod：exp_io0/1/2/3 = B16 A15 A13 B18
#   exp_io0 -> fan_pwm（4 线风扇 PWM 输入）
#   exp_io1 -> fan_tach（风扇转速反馈，外部上拉）
#   exp_io2 -> dht_dq（AM2302 单总线，外部 4.7k 上拉到 3.3V）
#   exp_io3 -> heat_pwm（环境加热电阻 PWM，低边 N-MOS 栅极，需 100R 串阻 + 10k 下拉）
#-----------------------------------------------------------------------
set_property -dict {PACKAGE_PIN B16 IOSTANDARD LVCMOS33} [get_ports fan_pwm]
set_property -dict {PACKAGE_PIN A15 IOSTANDARD LVCMOS33 PULLTYPE PULLUP} [get_ports fan_tach]
set_property -dict {PACKAGE_PIN A13 IOSTANDARD LVCMOS33 PULLTYPE PULLUP} [get_ports dht_dq]
set_property -dict {PACKAGE_PIN B18 IOSTANDARD LVCMOS33} [get_ports heat_pwm]

#-----------------------------------------------------------------------
# 数码管第 1 组（右 4 位）段选：D4 E3 D3 F4 F3 E2 D2 H2
#-----------------------------------------------------------------------
set_property -dict {PACKAGE_PIN D4 IOSTANDARD LVCMOS33} [get_ports {seg2[0]}]
set_property -dict {PACKAGE_PIN E3 IOSTANDARD LVCMOS33} [get_ports {seg2[1]}]
set_property -dict {PACKAGE_PIN D3 IOSTANDARD LVCMOS33} [get_ports {seg2[2]}]
set_property -dict {PACKAGE_PIN F4 IOSTANDARD LVCMOS33} [get_ports {seg2[3]}]
set_property -dict {PACKAGE_PIN F3 IOSTANDARD LVCMOS33} [get_ports {seg2[4]}]
set_property -dict {PACKAGE_PIN E2 IOSTANDARD LVCMOS33} [get_ports {seg2[5]}]
set_property -dict {PACKAGE_PIN D2 IOSTANDARD LVCMOS33} [get_ports {seg2[6]}]
set_property -dict {PACKAGE_PIN H2 IOSTANDARD LVCMOS33} [get_ports {seg2[7]}]
