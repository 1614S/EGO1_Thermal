# EGO1 芯片结温感知与动态热管理演示系统 —— 代码骨架 v0.2

基于 EGO1（**XC7A35T-1LCSG324I**，Vivado 2026.1）的闭环热管理演示工程：
XADC 结温 + AM2301 外部温湿度 → 统一采集 → 滤波/标定 → PI 控风扇 → MMCM DRP 五档调频 → UART 遥测。

v0.2 更新：
- **AM2301 已修复**：根因不是位对齐，而是 `400×6554` 被 16 bit 上下文截断；改成 32 bit 乘法后读数正确（40.0 ℃ / 55.0 %RH）。
- **引脚已按官方 `EGo1.xdc` 填好**（`constr/ego1_pins.xdc`）。
- **PI 乘法/符号宽度修复**：`p_c + i_c[31:16]` 曾因无符号扩展导致占空比误判，现在闭环稳态误差 0.6 ℃。
- **综合分支已通过 elaboration**（MMCME2_ADV / XADC / BUFG 原语 + glbl），不再是"只会在仿真里跑"。

## 快速开始

```bat
:: 1) 三个自检仿真
scripts\sim.bat tb_uart
scripts\sim.bat tb_ctrl
scripts\sim.bat tb_top_smoke

:: 2) 把源码加入现有工程 C:\Users\13692\Thermal
"D:\2026-1\FPGA\2026.1\Vivado\bin\vivado.bat" -mode batch -source scripts\add_to_project.tcl

:: 3) 综合 / 实现 / 出报告
"D:\2026-1\FPGA\2026.1\Vivado\bin\vivado.bat" -mode batch -source scripts\build.tcl
```

## 已验证（本机 2026.1 工具链）

| 项 | 结果 |
|---|---|
| 综合分支 elaboration | `xelab -L unisims_ver ego1_thermal_top glbl` → Built（原语端口/属性正确） |
| UART 回环 | 256/256 字节，0 错误 |
| 顶层遥测帧 | 99 帧，CRC 0 错；结温 45.04 ℃、外部 39.99 ℃（模型注入 45.0 / 40.0） |
| 档位 3→4 | `f_meas` = 1500（150.0 MHz），切换不复位 |
| PI 闭环 | 收敛 50.59 ℃，稳态误差 0.58 ℃，调节 20.5 s，超调 15 %（待整定） |

## 引脚映射（摘自官方 EGo1.xdc）

| 信号 | 引脚 |
|---|---|
| clk / rst_btn | P17 / P15 |
| btn[4:0] | R11 R17 R15 V1 U4（按下为低，顶层已取反） |
| sw[7:0] | P5 P4 P3 P2 R2 M4 N4 R1 |
| led[15:0] | F6 G4 G3 J4 H4 J3 J2 K2 K1 H6 H5 J5 K6 L1 M1 K3 |
| seg[7:0]（0 组段选） | B4 A4 A3 B1 A1 B3 B2 D5 |
| an[3:0]（0 组位选） | G2 C2 C1 H1 |
| uart_rx / uart_tx | N5 / T4 |
| fan_pwm / fan_tach / dht_dq | B16 / A15 / A13（Pmod exp_io0/1/2） |

> 数码管只用了 0 组（4 位）；若上板发现段码顺序或共阳/共阴极性不符，改 `seg7_scan` 的译码表或 XDC 里的位序即可。

## 已知问题 / 下一步

1. **PI 超调 15 %**（指标 ≤10 %）：仿真对象是一阶+延迟的近似，需上板后按真实热对象整定；可选加前馈 `duty_ff = k·(Tset−Tamb)`。
2. **待补模块**：`cmd_proc`（ASCII 命令）、`fan_tach`（4 线风扇测速）、`acq_fifo`、`dbg_mux`、ILA 约束。
3. **XADC 只读了温度通道**，VCCINT/VCCAUX 需要时再扩展（在当前单通道模式上改写 42h）。
4. **DRP 切档未经上板验证**：寄存器编码已按 Clocking Wizard 模板推导并用 PG065 复位值反验证，但首次上板建议先用 `tb_clk` 的思路做单步确认。

## 设计文档

- [架构与仿真方案](docs/01_架构与仿真方案.md)
- [上板调试记录模板](docs/02_上板调试.md)
- [测试记录模板](docs/03_测试记录.md)
