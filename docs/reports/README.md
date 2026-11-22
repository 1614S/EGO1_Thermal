# 综合 / 实现报告归档

本目录保存**可复现的**工程度量报告，作为赛题"时序与资源"指标的证据。

| 文件 | 内容 | 关键结论 |
|---|---|---|
| `util.rpt` | 实现后资源占用（`report_utilization`） | LUT 1113 / 20800 = **5.35 %**；FF 1059；DSP 6；BRAM 0 |
| `timing.rpt` | 实现后时序（`report_timing_summary`） | **WNS +0.340 ns**，TNS 0.000，0/2946 失败端点；WHS +0.058 ns |
| `power.rpt` | 功耗估算（`report_power`） | 总 0.217 W（动态 0.155 + 静态 0.062） |

## 生成环境

| 项 | 值 |
|---|---|
| 器件 | `xc7a35ticsg324-1L` |
| 工具 | Vivado 2026.1 (64-bit)，Build 6511674 |
| 工程文件 | `Thermal.xpr`（根目录） |
| 对应提交 | `c51ce89` — *fix: XADC 换算流水化修复时序违例；构建脚本不再依赖工作目录* |
| 生成时间 | 2026-11-15 14:08 |
| 比特流 | `Thermal.runs/impl_1/ego1_thermal_top.bit`，2 192 220 B，14:08:31 |
| 复现命令 | `scripts\build.bat` |

## 阅读要点

1. **这只是 v0.2 骨架的数据**。LUT 5.35 % 不代表最终规模：自热负载只有 16 路 LFSR，
   `cmd_proc`、加热闭环等已加入；采集 FIFO、ILA 仍未做。文档里 7.4 k LUT（35 %）是早期完整版估算，实测为 4749 LUT（22.8 %）。
2. **WNS 余量仅 0.34 ns**。改动 `clk_sys` 域（尤其 `reg_file`、`frame_pack`、显示译码）后
   必须重跑本报告；`clk_core` 域按 150 MHz（6.667 ns）约束，余量约 5 ns。
3. 上一版曾出现 **WNS −0.966 ns**：XADC `code×63/2 − 69926` 的单周期组合链
   （DSP48 + 7 级 CARRY4 = 10.9 ns）超时。已拆成两级流水（`S_DRDY` 打拍 → `S_CONV` 饱和输出），
   这是本报告 WNS 转正的原因。
4. 功耗 0.217 W 是当前骨架的估算值，**远低于热管理演示所需**（目标 1–2 W）；
   上板前需按 `heat_load_gen` 的实际规模重新评估，不能直接引用这个数。

## 更新方式

```bat
scripts\build.bat
copy results\timing.rpt docs\reports\timing.rpt
copy results\util.rpt    docs\reports\util.rpt
copy results\power.rpt   docs\reports\power.rpt
```

改动较大时建议在提交说明里注明报告对应的 commit，便于追溯。