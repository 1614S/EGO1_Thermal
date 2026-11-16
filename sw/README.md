# sw/ —— 上位机与数据分析

## analyze_thermal.py

热响应分析：τ（热时间常数）、θ（热阻 + 不确定度）、阶跃指标（超调/调节时间/稳态误差），
并按报告要求出 SVG 曲线。

**只用标准库**（没有 numpy/scipy/matplotlib），原因是"别人机器上装不装得起"不可控；
SVG 是纯文本，浏览器、VSCode、Word 都能直接看，需要位图时再转。

### 用法

```bat
:: 1) 自检（不需要任何数据文件）
python sw\analyze_thermal.py --selftest

:: 2) 用实测 CSV 出 τ 和阶跃指标
python sw\analyze_thermal.py --csv results\step_50C.csv --out figs

:: 3) 顺便算热阻（给环境温度与功率）
python sw\analyze_thermal.py --csv results\step_50C.csv --out figs --tamb 25 --p-mw 1200

:: 4) 造演示数据，验证流程
python sw\analyze_thermal.py --demo --out figs --tamb 25 --p-mw 1200
```

### 自检输出（可直接写进报告）

```
T1 tau fit       : true 18.00 s  ->  fitted 17.99 s (0.04% err, R2=0.9998)
T2 tau offset    : fitted 18.00 s (0.00% err)
T3 step metrics  : overshoot 0.7%  settle 54.5 s  ess 0.01 degC
T4 theta         : 20.83 +/- 5.24 degC/W (rel 25.2%)
T5 csv roundtrip : 400 rows
T6 svg           : work\_selftest.svg
[PASS] analyze_thermal selftest
```

### CSV 列约定

列名大小写、单位宽松匹配，缺列不致命：

| 列 | 别名 | 说明 |
|---|---|---|
| `t_s` | `ts_ms`（自动 /1000）、`t`、`time` | 时间（秒） |
| `tj_c` | `tj`、`temp` | 结温（℃） |
| `tset_c` | `tset`、`setpoint` | 设定值（℃） |
| `duty` | `duty_pm` | 占空比（0–1000 或 0–100，自动识别） |
| `gear` | `gear_act` | 档位 |
| `p_mw` | `p`、`power` | 功率（mW） |

示例：

```csv
t_s,tj_c,tset_c,duty,gear,p_mw
0.50,32.10,50.0,12,3,1200
1.00,32.35,50.0,25,3,1200
```

### 方法说明（报告里要写）

- **τ 拟合**：`T(t) = T∞ − (T∞ − T0)·exp(−(t−t0)/τ)`。
  固定 τ 时对 (A, B) 是线性最小二乘，只需对 τ 做一维搜索（对数轴粗扫 + 黄金分割细化），
  不用 scipy，也不会陷局部极小。
- **95 % 区间**：用剖面似然近似（`SSE(τ) ≤ SSE_min·(1+4/n)`），报告里须注明是近似值，
  不是 bootstrap。
- **θ**：`θ = ΔT/P`，`u(θ)/θ = √((u(ΔT)/ΔT)² + (u(P)/P)²)`，默认 `u(ΔT)=0.71 ℃`（两路各 0.5 ℃）、
  `u(P)/P=25 %`（XPE + 翻转率估计）。**主导项是功耗估计误差，不是温度传感器** ——
  这是 §f 的核心结论。

### 待补

- ~~串口采集脚本~~ → 已完成，见 `host_logger.py`（32 B 帧解码 + CRC 校验 + CSV 落盘，
  输出列与 `analyze_thermal.py` 对齐）。上板采集纪律见 `docs/05_实测数据采集说明.md`。
