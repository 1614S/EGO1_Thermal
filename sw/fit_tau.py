#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fit_tau.py —— 热时间常数 τ 与热阻 θ 估计（板级实测，含不确定度与置信区间）

适用数据：scripts/capture_serial.ps1 或 sw/host_logger.py 输出的遥测 CSV
          （新格式列：t_s, tj_c, text_c, rh_pct, tset_c, duty, gear, state,
            f_mhz, p_mw, age, flags, seq, tset_ext_c, duty_heat）

两路通道通用：
    --channel tj    结温通道（XADC，列 tj_c）      —— 默认
    --channel text  传感器通道（AM2302，列 text_c）

环境温度 Tamb 的取法（按要求）：
    * 默认取**传感器温度的初值** text_c[0]（AM2302 在扰动开始前的读数）；
    * 用 --tamb 可显式覆盖（例如另用温度计读数）。

θ 的定义与算法：
    θ = (T0 − Tamb) / P
    T0  = 扰动前起点稳态（前 --t0-win 秒的中位数，默认 10 s）
    P   = 自热功耗，默认取 CSV 的 p_mw，可用 --p-mw 覆盖

τ 的算法：
    自动找峰值 → 对峰后降温段拟合 y = A + B·exp(−t/τ)
    给定 τ 时对 A、B 做线性最小二乘；对 τ 先粗网格（对数等距）再局部细化。

不确定度（GUM，k=1 合成 / k=2 扩展 ≈95%）：
    u(θ)/θ = sqrt( (u_P/P)² + (u_T0² + u_Tamb²) / (T0−Tamb)² )
    u(τ) 用**残差自助法**（bootstrap）给出 95% 百分位置信区间。

输出：在 results/ 下新建子目录 <stem>_<channel>_<时间戳>/，内含
    report.md   估算结果 + 不确定度分析 + 置信区间 + 方法与复现命令
    curve.png   温度-时间曲线（含拟合曲线、峰值、τ 切线）
    fit.json    机器可读结果
    <原CSV>     原始数据副本

用法：
    python sw/fit_tau.py --csv results/tau_theta_run.csv
    python sw/fit_tau.py --csv results/x.csv --channel text
    python sw/fit_tau.py --csv results/x.csv --p-mw 750 --tamb 22.9
"""

import argparse
import csv
import json
import math
import os
import shutil
from datetime import datetime
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# 数据读取
# ---------------------------------------------------------------------------
def load_csv(path):
    """返回 dict：列名 → float 数组（缺列自动跳过）。兼容带 BOM 的 CSV。"""
    data = {}
    with open(path, encoding="utf-8-sig", newline="") as f:
        rd = csv.DictReader(f)
        cols = [c for c in (rd.fieldnames or [])]
        rows = list(rd)
    if not rows:
        raise SystemExit(f"CSV 是空的：{path}")
    for c in cols:
        vals = []
        ok = True
        for r in rows:
            v = r.get(c, "")
            try:
                vals.append(float(v))
            except (TypeError, ValueError):
                ok = False
                break
        if ok:
            data[c] = np.asarray(vals, dtype=float)
    if "t_s" not in data:
        raise SystemExit(f"CSV 缺少 t_s 列：{path}")
    return data


def pick_series(data, channel):
    col = "tj_c" if channel == "tj" else "text_c"
    if col not in data:
        raise SystemExit(f"CSV 里没有 {col} 列（--channel {channel}）")
    return col, data[col]


def series_power(data, override):
    if override is not None:
        return float(override), "--p-mw 指定"
    if "p_mw" in data:
        return float(np.median(data["p_mw"])), "CSV p_mw 中位数"
    raise SystemExit("CSV 没有 p_mw，且未给 --p-mw")


def series_tamb(data, override, channel):
    """环境温度 Tamb。

    * 结温通道（tj）：默认取**传感器温度初值** text_c[0]（扰动开始前 AM2302 读到的室温）；
    * 传感器通道（text）：被拟合的就是 text_c 本身，它的初值是"自热稳态"而不是环境温度，
      必须用 --tamb 显式给出（可用第二个温度计，或另一个未被加热的传感器）。
    """
    if override is not None:
        return float(override), "--tamb 指定"
    if channel == "text":
        raise SystemExit(
            "text 通道必须用 --tamb 指定环境温度：\n"
            "  被拟合的 text_c 初值是传感器自身的（自热）稳态，不是环境温度。\n"
            "  例如：--tamb 23.0（取当日室温/未受扰动的另一个温度计读数）"
        )
    if "text_c" in data:
        return float(data["text_c"][0]), "CSV text_c 初值（AM2302，扰动前室温）"
    raise SystemExit("CSV 没有 text_c，且未给 --tamb")


# ---------------------------------------------------------------------------
# τ 拟合：网格搜索 + 线性最小二乘
# ---------------------------------------------------------------------------
def _fit_linear(t, y, tau):
    e = np.exp(-t / tau)
    X = np.column_stack([np.ones_like(e), e])
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    A, B = float(coef[0]), float(coef[1])
    pred = A + B * e
    resid = y - pred
    sse = float(np.sum(resid ** 2))
    return A, B, sse, pred, resid


def fit_tau(t, y, lo=0.2, hi=900.0, n_coarse=240, n_fine=120):
    taus = np.geomspace(lo, hi, n_coarse)
    best = None
    for tau in taus:
        A, B, sse, pred, resid = _fit_linear(t, y, tau)
        if best is None or sse < best["sse"]:
            best = dict(tau=float(tau), A=A, B=B, sse=sse, pred=pred, resid=resid)
    # 局部细化
    tau0 = best["tau"]
    for tau in np.linspace(tau0 * 0.8, tau0 * 1.25, n_fine):
        if tau <= 0:
            continue
        A, B, sse, pred, resid = _fit_linear(t, y, tau)
        if sse < best["sse"]:
            best = dict(tau=float(tau), A=A, B=B, sse=sse, pred=pred, resid=resid)

    n = len(y)
    sst = float(np.sum((y - y.mean()) ** 2))
    best["r2"] = 1.0 - best["sse"] / sst if sst > 0 else float("nan")
    best["n"] = n
    best["resid_std"] = float(np.std(best["resid"], ddof=2)) if n > 2 else float("nan")
    return best


def bootstrap_tau(t, y, fit, n_boot=400, seed=12345):
    """残差自助法：给 τ 的 95% 百分位置信区间。"""
    rng = np.random.default_rng(seed)
    r = fit["resid"]
    base = fit["pred"]
    taus = []
    for _ in range(n_boot):
        yb = base + rng.choice(r, size=len(r), replace=True)
        try:
            fb = fit_tau(t, yb, n_coarse=80, n_fine=40)
            taus.append(fb["tau"])
        except Exception:
            continue
    if len(taus) < 20:
        return None
    arr = np.asarray(taus)
    return dict(
        n=len(arr),
        median=float(np.median(arr)),
        lo=float(np.percentile(arr, 2.5)),
        hi=float(np.percentile(arr, 97.5)),
        std=float(np.std(arr, ddof=1)),
    )


def t63_crossing(t, y, fit):
    """t63：曲线从峰值降到 (峰值−稳态)*e^-1 处的时间。"""
    A = fit["A"]                      # 稳态渐近值
    y0 = y[0]
    target = A + (y0 - A) * math.exp(-1.0)
    for ti, yi in zip(t, y):
        if yi <= target:
            return float(ti)
    return float("nan")


def detect_mode(y, noise):
    """自动判别数据形状，决定拟合哪一段。

    * "disturbance" —— 外部扰动：中间有明显峰值，两侧都低于峰值 → 只拟合峰后段
    * "step"        —— 设定值阶跃：从一端单调走到另一端 → 拟合整段
    * "unknown"     —— 两者都不像，交给上层拒绝
    """
    n = len(y)
    if n < 60:
        return "unknown", dict(reason=f"只有 {n} 帧，太短")
    y0, y1 = float(y[0]), float(y[-1])
    imax = int(np.argmax(y))
    span = float(y.max() - y.min())
    thr = max(0.5, 5.0 * noise)

    # A. 中间有峰 → 扰动实验
    if (0.08 * n) < imax < (0.92 * n) and (y[imax] - max(y0, y1)) > thr:
        return "disturbance", dict(peak_idx=imax, peak=float(y[imax]), span=span)

    # B. 单调阶跃（升或降）
    #    用**分块中位数**判趋势：慢衰减 + 噪声时逐点差分符号随机，
    #    但每块的温度中位数会稳稳地朝一个方向走（踩过：逐点判法把阶跃误判成 unknown）
    total = y1 - y0
    k_blocks = int(max(4, min(12, n // 20)))
    edges = np.linspace(0, n, k_blocks + 1).astype(int)
    med = np.array([np.median(y[edges[i]:max(edges[i] + 1, edges[i + 1])])
                    for i in range(k_blocks)])
    dd = np.diff(med)
    bad = int(np.sum((np.sign(dd) != np.sign(total)) & (np.abs(dd) > 0.05)))
    mono = 1.0 - bad / max(1, len(dd))
    if abs(total) > thr:
        if mono >= 0.85 and span >= 0.8 * abs(total):
            return "step", dict(peak_idx=(0 if total < 0 else n - 1),
                                peak=float(y[0] if total < 0 else y[-1]),
                                span=span, monotonic=mono, delta=float(total))


    return "unknown", dict(reason=f"既不像中间有峰的扰动，也不像单调阶跃"
                                  f"（首尾差 {total:.2f} K，分块单调占比 {mono*100:.0f} %）")


def rise_ratio(y_peak, tamb, y0):
    """这次扰动把温度抬了多少（相对起点）。"""
    return y_peak - y0


# ---------------------------------------------------------------------------
# 绘图（matplotlib，英文标注避免缺字形）
# ---------------------------------------------------------------------------
def plot_curve(path, t_all, y_all, t_fit, y_fit, fit, meta, tau_ci=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9.6, 5.2), dpi=130)
    ax.plot(t_all, y_all, lw=1.0, color="#1f4e79",
            label=f"measured ({meta['channel_col']})")
    ax.plot(t_fit, y_fit, lw=1.8, color="#c00000",
            label=f"fit  $y=A+B\\,e^{{-t/\\tau}}$,  $\\tau={fit['tau']:.2f}$ s  ($R^2={fit['r2']:.4f}$)")
    ax.axhline(meta["A"], ls="--", lw=1.0, color="#9a9a9a",
               label=f"fit asymptote  $A$ = {meta['A']:.2f} $^\\circ$C")

    # 峰值标记（标注放右侧，避开标题）
    ip = meta["ip"]
    _lbl = "start" if meta.get("mode") == "step" else "peak"
    ax.plot([t_all[ip]], [y_all[ip]], "o", ms=5, color="#c00000", zorder=5)
    ax.annotate(f"{_lbl} {y_all[ip]:.2f} $^\\circ$C @ {t_all[ip]:.1f} s",
                xy=(t_all[ip], y_all[ip]),
                xytext=(t_all[ip] + 0.10 * (t_all[-1] - t_all[0]), y_all[ip] + 0.10),
                fontsize=8, color="#c00000",
                arrowprops=dict(arrowstyle="->", lw=0.8, color="#c00000"))

    # τ 位置（标注放底部，避免和峰值标注打架）
    tp = meta["t_peak"]
    t1 = tp + fit["tau"]
    ax.axvline(t1, ls="-.", lw=0.9, color="#2e7d32", alpha=0.9)
    _ylo, _yhi = ax.get_ylim()
    ax.annotate(f"$t_0+\\tau$ = {t1 - t_all[0]:.1f} s",
                xy=(t1, _ylo + 0.12 * (_yhi - _ylo)),
                xytext=(3, 0), textcoords="offset points", va="bottom", ha="left",
                fontsize=8, color="#2e7d32")

    if tau_ci:
        ax.axvspan(tp + tau_ci["lo"], tp + tau_ci["hi"], color="#2e7d32", alpha=0.15,
                   label=f"$\\tau$ 95% CI = [{tau_ci['lo']:.1f}, {tau_ci['hi']:.1f}] s")

    # 纵轴只显示数据范围；Tamb 用文字说明，避免被拉到 22 ℃ 把曲线压扁
    y_lo = float(min(y_all))
    y_hi = float(max(y_all))
    pad = max((y_hi - y_lo) * 0.18, 0.3)
    ax.set_ylim(y_lo - pad * 0.5, y_hi + pad)
    ax.set_xlim(t_all[0] - 2, t_all[-1] + 2)
    ax.text(0.012, 0.03,
            f"$T_{{amb}}$ = {meta['tamb']:.2f} $^\\circ$C  (sensor initial value)",
            transform=ax.transAxes, fontsize=8, color="#555555")

    ax.set_xlabel("time t (s)")
    ax.set_ylabel("temperature ($^\\circ$C)")
    ax.set_title(f"Thermal time-constant fit  |  {meta['csv_name']}  |  channel = {meta['channel']}",
                 fontsize=11)
    ax.grid(alpha=0.25)
    fig.tight_layout(rect=(0, 0.10, 1, 1))
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=2, fontsize=7.8,
               frameon=False, bbox_to_anchor=(0.5, 0.005))
    fig.savefig(path)
    plt.close(fig)


# ---------------------------------------------------------------------------
# 报告
# ---------------------------------------------------------------------------
def write_report(path, meta, theta, u_rel, u_terms, tau_ci, fit, t63):
    L = []
    A = L.append
    A(f"# τ / θ 估算报告 —— {meta['csv_name']}（{meta['channel']} 通道）")
    A("")
    A(f"- 生成时间：{meta['when']}")
    A(f"- 数据文件：`{meta['csv_name']}`（{meta['n']} 帧，{meta['t_span']:.1f} s）")
    A(f"- 通道：`{meta['channel']}` → 列 `{meta['channel_col']}`")
    _mode_cn = {"disturbance": "外部扰动（拟合峰后段）",
                "step": "设定值阶跃（拟合整段）"}.get(meta.get("mode"), "未判别")
    A(f"- 数据形状：**{_mode_cn}**")
    A(f"- 复现命令：")
    A("")
    A("```bash")
    A(meta["cmd"])
    A("```")
    A("")
    A("## 1. 结果")
    A("")
    A("| 量 | 数值 | 说明 |")
    A("|---|---|---|")
    A(f"| 起点稳态 T0 | **{meta['t0']:.2f} ℃** | 前 {meta['t0_win']:.0f} s 中位数 |")
    A(f"| 环境温度 Tamb | **{meta['tamb']:.2f} ℃** | {meta['tamb_src']} |")
    A(f"| 自热功耗 P | **{meta['p_mw']:.0f} mW** | {meta['p_src']} |")
    A(f"| **热阻 θ** | **{theta:.2f} ± {2*u_rel*theta:.2f} ℃/W** | k=2（≈95%），相对 {u_rel*100:.1f} % |")
    if meta.get("mode") == "step":
        A(f"| 阶跃 | {meta['t_peak']:.1f} s 处开始，变化 {meta['peak']-meta['t0']:+.2f} K | 整段拟合（阶跃响应） |")
    else:
        A(f"| 峰值 | {meta['peak']:.2f} ℃ @ {meta['t_peak']:.1f} s | 相对起点 +{meta['peak']-meta['t0']:.2f} K |")
    A(f"| **时间常数 τ（拟合）** | **{fit['tau']:.2f} s** | R² = {fit['r2']:.4f} |")
    if tau_ci:
        A(f"| τ 的 95% 置信区间 | **[{tau_ci['lo']:.2f}, {tau_ci['hi']:.2f}] s** | 残差自助法，{tau_ci['n']} 次重采样 |")
        A(f"| τ 的标准差 u(τ) | {tau_ci['std']:.2f} s（相对 {tau_ci['std']/fit['tau']*100:.1f} %） | 同上 |")
    A(f"| τ（t63 法） | {t63:.2f} s | 与拟合值相差 {abs(t63-fit['tau'])/((t63+fit['tau'])/2)*100:.1f} % |")
    A(f"| 拟合参数 | A = {fit['A']:.3f}，B = {fit['B']:.3f} | y = A + B·exp(−t/τ) |")
    A(f"| 残差标准差 | {fit['resid_std']:.3f} ℃ | 拟合质量 |")
    A("")
    A("![温度-时间曲线与拟合](curve.png)")
    A("")
    A("## 2. 不确定度分析（GUM）")
    A("")
    A("θ = (T0 − Tamb) / P，按相对不确定度合成：")
    A("")
    A("$$\\frac{u(\\theta)}{\\theta}=\\sqrt{\\left(\\frac{u_P}{P}\\right)^2"
      "+\\frac{u_{T0}^2+u_{T_{amb}}^2}{(T_0-T_{amb})^2}}$$")
    A("")
    A("| 来源 | 假设值 | 对 θ 的相对贡献 |")
    A("|---|---|---|")
    A(f"| 功耗 P | ±{u_terms['u_p_rel']*100:.1f} % | {u_terms['u_p_rel']*100:.1f} % |")
    A(f"| 温差 ΔT = T0−Tamb = {meta['t0']-meta['tamb']:.2f} K | ±{u_terms['u_dt']:.2f} K | {u_terms['u_dt_rel']*100:.1f} % |")
    A(f"| **合成 u_c(θ)/θ（k=1）** | — | **{u_rel*100:.1f} %** |")
    A(f"| **扩展 U（k=2，≈95%）** | — | **{2*u_rel*100:.1f} %** → ±{2*u_rel*theta:.2f} ℃/W |")
    A("")
    A(f"- `u_P = {u_terms['u_p_rel']*100:.0f} %`：功耗来自板级功耗模型（P = f(V²·C·f + 静态）），"
      f"未做电流实测；这是**主导项**，可用 --u-p 调整。")
    A(f"- `u_T = {u_terms['u_temp']:.2f} K`：包含传感器精度（XADC 片内 ±1 ℃ 量级 / AM2302 ±0.5 ℃）"
      f"与读数噪声（窗口内标准差 {u_terms['noise']:.3f} K），按 --u-temp 可调。")
    A(f"- τ 的不确定度单独用自助法给出（见上表），不参与 θ 的合成（θ 与 τ 独立估计）。")
    A("")
    A("## 3. 方法与假设")
    A("")
    A("1. **一阶热模型**：`C dT/dt = P − (T−Tamb)/θ`，解为 `T(t) = Tamb + Pθ(1−e^{−t/τ})`，τ = θ·C。")
    A("   本实验用**外部热扰动**（靠近热源再移开）替代阶跃功耗，峰后降温段满足 `y = A + B e^{−t/τ}`。")
    A("2. **θ 用起点稳态**：扰动前系统已在 P 下稳态，`θ = (T0−Tamb)/P`。")
    A("3. **Tamb 取传感器初值**：`text_c[0]`（AM2302 在扰动前的读数），避免用扰动期间被加热的空气温度。")
    A("4. **τ 用峰后段拟合**：峰值前受扰动源位置/风场影响，不符合一阶模型；峰后自由降温段才有效。")
    A("5. **单侧扰动**：只加热不制冷，因此只能覆盖 τ 的降温侧；升温侧 τ 可能略有差异（对流系数随温差变化）。")
    A("")
    A("## 4. 结论与使用建议")
    A("")
    A(f"- 本次实测：**τ ≈ {fit['tau']:.1f} s、θ ≈ {theta:.1f} ℃/W**（{meta['channel']} 通道）。")
    A(f"- 不确定度以功耗项为主（{u_terms['u_p_rel']*100:.0f} %），若要把 θ 做到 ±10 % 以内，"
      f"必须实测电流/电压而不是用功耗模型。")
    A("- 报告与曲线、原始 CSV 已归档在同一目录，可直接作为 `docs/05_证据表.md` 第 17/18 项的附件。")
    A("")
    A("---")
    A("")
    A("## 附：目录内容")
    A("")
    for fn in meta["files"]:
        A(f"- `{fn}`")
    A("")
    Path(path).write_text("\n".join(L), encoding="utf-8")


# ---------------------------------------------------------------------------
def synth_csv(path, tau_true, theta_true, p_mw, tamb, t_off, dur, dt=0.1,
              noise=0.02, seed=7, p_bump=0.16, t_on=20.0, column="tj"):
    """生成一份"已知 τ/θ"的合成遥测 CSV，用于 --selftest 验证脚本本身。"""
    rng = np.random.default_rng(seed)
    p_w = p_mw / 1000.0
    c_th = tau_true / theta_true            # J/K
    t = np.arange(0.0, dur, dt)
    T = tamb + p_w * theta_true          # 从自热稳态起步：T0 = Tamb + P*theta
    ys = []
    for ti in t:
        # 扰动期间在自热功耗上叠加一个额外热源
        p = p_w + (p_bump if t_on <= ti < t_off else 0.0)
        T = T + (p / c_th - (T - tamb) / (theta_true * c_th)) * dt
        ys.append(T + rng.normal(0.0, noise))
    ys = np.asarray(ys)
    cols = ["t_s", "tj_c", "text_c", "rh_pct", "tset_c", "duty", "gear", "state",
            "f_mhz", "p_mw", "age", "flags", "seq", "tset_ext_c", "duty_heat"]
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for i, ti in enumerate(t):
            tj_v, tx_v = (ys[i], tamb) if column == "tj" else (tamb, ys[i])
            w.writerow([f"{ti:.3f}", f"{tj_v:.4f}", f"{tx_v:.4f}", "50.0", "50.00",
                        "500", "3", "0", "100.0", f"{p_mw:.0f}", "0", "0", str(i),
                        "40.00", "500"])
    return path


def synth_step_csv(path, tau_true, y_start, y_end, dur, dt=0.1, noise=0.02,
                   seed=11, tamb=23.0):
    """生成"设定值阶跃"形状的 CSV：y = y_end + (y_start-y_end)*exp(-t/tau)。"""
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, dur, dt)
    y = y_end + (y_start - y_end) * np.exp(-t / tau_true) + rng.normal(0.0, noise, len(t))
    cols = ["t_s", "tj_c", "text_c", "rh_pct", "tset_c", "duty", "gear", "state",
            "f_mhz", "p_mw", "age", "flags", "seq", "tset_ext_c", "duty_heat"]
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for i, ti in enumerate(t):
            w.writerow([f"{ti:.3f}", f"{y[i]:.4f}", f"{tamb:.4f}", "50.0", "33.00",
                        "500", "3", "0", "100.0", "750", "0", "0", str(i),
                        "40.00", "500"])
    return path


def run_selftest():
    """合成已知参数的数据，验证：扰动（tj/text）与阶跃三种情形都能复原 τ。"""
    import tempfile

    cases = [
        # (名字, 通道, tau真值, theta真值, P_mW, Tamb, 扰动结束时刻, 总时长)
        ("disturb-tj",   "tj",   21.91, 20.0, 750, 22.99, 50.0, 140),
        ("disturb-text", "text", 45.00, 30.0, 750, 23.00, 70.0, 320),
    ]
    ok = True
    print("=== fit_tau.py 自检（合成数据，已知真值）===")
    with tempfile.TemporaryDirectory() as td:
        for name, ch, tau_t, th_t, p_mw, tamb, t_off, dur in cases:
            p = Path(td) / f"synth_{ch}.csv"
            synth_csv(p, tau_t, th_t, p_mw, tamb, t_off, dur, column=ch)
            try:
                res = estimate(p, ch, p_mw_override=p_mw, tamb_override=tamb,
                               u_p=0.25, u_temp=0.5, t0_win=10.0, n_boot=120,
                               quiet=True, out_dir=Path(td) / f"out_{ch}")
            except SystemExit as e:
                print(f"[FAIL] {ch}: 估计流程拒绝了合成数据（{e}）")
                ok = False
                continue
            e_tau = abs(res["tau_s"] - tau_t) / tau_t * 100
            e_th = abs(res["theta_C_per_W"] - th_t) / th_t * 100
            good = e_tau <= 10 and e_th <= 15
            ok = ok and good
            print(f"[{'PASS' if good else 'FAIL'}] {name:12s} (通道 {ch:4s}): "
                  f"tau {res['tau_s']:.2f} s (真值 {tau_t:.2f}, 误差 {e_tau:.1f} %)  "
                  f"theta {res['theta_C_per_W']:.2f} C/W (真值 {th_t:.2f}, 误差 {e_th:.1f} %)  "
                  f"R^2={res['tau_r2']:.4f} mode={res['mode']}")

        # --- 阶跃用例：只要能识别出 step 并复原 tau 即可（θ 在阶跃下无物理意义）---
        tau_s = 60.0
        p_step = Path(td) / "synth_step.csv"
        synth_step_csv(p_step, tau_s, 36.3, 33.3, 240)
        try:
            res = estimate(p_step, "tj", p_mw_override=750, tamb_override=30.8,
                           u_p=0.25, u_temp=0.5, t0_win=10.0, n_boot=120,
                           quiet=True, out_dir=Path(td) / "out_step")
            e_t = abs(res["tau_s"] - tau_s) / tau_s * 100
            good = (res["mode"] == "step") and e_t <= 10
            ok = ok and good
            print(f"[{'PASS' if good else 'FAIL'}] {'step':12s} (通道 tj  ): "
                  f"tau {res['tau_s']:.2f} s (真值 {tau_s:.2f}, 误差 {e_t:.1f} %)  "
                  f"mode={res['mode']}  R^2={res['tau_r2']:.4f}")
        except SystemExit as e:
            print(f"[FAIL] step: 被拒绝（{e}）")
            ok = False
    print("=== 自检" + ("通过" if ok else "失败") + " ===")
    return 0 if ok else 1


def estimate(csv_path, channel, p_mw_override=None, tamb_override=None,
             u_p=0.25, u_temp=0.5, t0_win=10.0, n_boot=400, out_dir=None,
             quiet=False):
    """跑完整流程：读数据 → 校验 → 拟合 → 不确定度 → 报告/曲线/JSON。

    返回结果 dict（同时把 report.md / curve.png / fit.json / 原 CSV 写到 out_dir）。
    """
    csv_path = Path(csv_path).resolve()
    if not csv_path.exists():
        raise SystemExit(f"找不到 CSV：{csv_path}")
    data = load_csv(csv_path)

    channel_col, y_all = pick_series(data, channel)
    t_all = data["t_s"]
    p_mw, p_src = series_power(data, p_mw_override)
    tamb, tamb_src = series_tamb(data, tamb_override, channel)

    # 起点稳态（窗口取中位数，抗毛刺）
    dt = float(np.median(np.diff(t_all))) if len(t_all) > 1 else 0.1
    n0 = max(3, int(round(t0_win / max(dt, 1e-6))))
    n0 = min(n0, len(y_all) // 4 + 1)
    t0 = float(np.median(y_all[:n0]))
    noise = float(np.std(y_all[:n0], ddof=1)) if n0 > 1 else 0.0
    p_w = p_mw / 1000.0
    theta = (t0 - tamb) / p_w

    n = len(y_all)
    mode, minfo = detect_mode(y_all, noise)

    # ------------------------------------------------------------------
    # 按数据形状选段（这一步保证"阶跃数据不会被误拒"）
    #   扰动数据 → 峰后段（升温段不符合一阶模型）
    #   阶跃数据 → 整段（一阶系统对阶跃的整个响应就是一条指数）
    # ------------------------------------------------------------------
    problems = []
    if mode == "disturbance":
        seg_start = int(minfo["peak_idx"])
        if (n - seg_start) < 50:
            problems.append(f"峰后只有 {n - seg_start} 帧（<50），不足以拟合 τ")
        theta_ref = "起点稳态 T0（扰动前）"
        theta_t = t0
    elif mode == "step":
        dev = np.abs(y_all - y_all[0])
        hit = dev > max(2.0 * noise, 0.05)
        seg_start = int(np.argmax(hit)) if np.any(hit) else 0
        if (n - seg_start) < 50:
            problems.append(f"阶跃后只有 {n - seg_start} 帧（<50），不足以拟合 τ")
        theta_ref = "末值稳态（阶跃终值，与 analyze_thermal 口径一致）"
        theta_t = float(np.mean(y_all[-max(10, n // 20):]))
    else:
        problems.append("数据形状无法判别：" + str(minfo.get("reason", "")))
        seg_start = 0
        theta_ref = "—"
        theta_t = t0

    if mode != "unknown":
        theta = (theta_t - tamb) / p_w
        if abs(theta_t - tamb) < 0.2:
            problems.append(f"稳态 {theta_t:.2f} ℃ 与环境温度 {tamb:.2f} ℃ 几乎相等，θ 无意义")

    if problems:
        print(f"[拒绝] {csv_path.name}（{channel} 通道）：")
        for q in problems:
            print(f"  - {q}")
        print("")
        print("提示：")
        print("  * 外部扰动：升温 ≥1 K 后自然回落，采到回落稳定（2~5 分钟）")
        print("  * 设定值阶跃：改 S= 后一直采到稳定（≥3~5 个 τ）")
        print("  * 传感器通道：需要把 AM2302 单独加热后再自然冷却")
        raise SystemExit(2)

    t2 = t_all[seg_start:] - t_all[seg_start]
    y2 = y_all[seg_start:]
    ip = seg_start          # 绘图/报告用：扰动=峰值位置，阶跃=分析起点

    fit = fit_tau(t2, y2)
    tau_ci = bootstrap_tau(t2, y2, fit, n_boot=n_boot) if n_boot > 0 else None
    t63 = t63_crossing(t2, y2, fit)

    # 不确定度
    u_dt = math.sqrt(2.0) * u_temp
    u_dt_rel = u_dt / abs(theta_t - tamb) if abs(theta_t - tamb) > 1e-9 else float("inf")
    u_rel = math.sqrt(u_p ** 2 + u_dt_rel ** 2)
    u_terms = dict(u_p_rel=u_p, u_dt=u_dt, u_dt_rel=u_dt_rel, u_temp=u_temp, noise=noise)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    od = Path(out_dir) if out_dir else (csv_path.parent / f"{csv_path.stem}_{channel}_{stamp}")
    od.mkdir(parents=True, exist_ok=True)
    shutil.copy2(csv_path, od / csv_path.name)

    meta_png = dict(channel=channel, channel_col=channel_col, tamb=tamb,
                    A=fit["A"], ip=ip, t_peak=float(t_all[ip]), csv_name=csv_path.name,
                    mode=mode)
    plot_curve(od / "curve.png", t_all, y_all, t2 + t_all[ip], fit["pred"], fit,
               meta_png, tau_ci)

    res = dict(
        csv=str(csv_path), channel=channel, column=channel_col, mode=mode,
        n=int(n), duration_s=float(t_all[-1] - t_all[0]),
        T0=t0, Tamb=tamb, Tamb_source=tamb_src, P_mW=p_mw, P_source=p_src,
        theta_C_per_W=theta, u_rel=u_rel, U95_C_per_W=2 * u_rel * theta,
        tau_s=fit["tau"], tau_r2=fit["r2"], tau_A=fit["A"], tau_B=fit["B"],
        tau_resid_std=fit["resid_std"], tau_t63=t63,
        tau_ci95=[tau_ci["lo"], tau_ci["hi"]] if tau_ci else None,
        tau_u_std=tau_ci["std"] if tau_ci else None,
        u_p_rel=u_p, u_temp=u_temp, noise_K=noise,
        generated=datetime.now().isoformat(timespec="seconds"),
        out_dir=str(od),
    )
    (od / "fit.json").write_text(json.dumps(res, ensure_ascii=False, indent=2), encoding="utf-8")

    meta = dict(
        csv_name=csv_path.name, channel=channel, channel_col=channel_col,
        when=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        n=n, t_span=float(t_all[-1] - t_all[0]),
        t0=t0, t0_win=t0_win, tamb=tamb, tamb_src=tamb_src,
        p_mw=p_mw, p_src=p_src, peak=float(y_all[ip]), t_peak=float(t_all[ip]),
        A=fit["A"], mode=mode, files=["report.md", "curve.png", "fit.json", csv_path.name],
        cmd=f"python sw/fit_tau.py --csv {csv_path} --channel {channel} "
            f"--p-mw {p_mw:.0f} --tamb {tamb:.2f}",
    )
    write_report(od / "report.md", meta, theta, u_rel, u_terms, tau_ci, fit, t63)

    if not quiet:
        print(f"数据      : {csv_path.name}  ({n} 帧, {meta['t_span']:.1f} s)  通道 {channel}")
        print(f"起点稳态  : T0 = {t0:.2f} C（前 {t0_win:.0f} s 中位数）")
        print(f"环境温度  : Tamb = {tamb:.2f} C（{tamb_src}）")
        print(f"自热功耗  : P = {p_mw:.0f} mW（{p_src}）")
        print(f"热阻      : theta = {theta:.2f} +- {2*u_rel*theta:.2f} C/W (k=2, rel {u_rel*100:.1f} %)")
        if mode == "step":
            print("            注意：阶跃模式下 θ 仅供参考——若阶跃改变的是风扇转速/档位，")
            print("                  θ_eff 本身已变化，此时 θ 不代表器件热阻")
        print(f"数据形状  : {mode}（'disturbance'=扰动拟合峰后段 / 'step'=阶跃拟合整段）")
        print(f"{'峰值' if mode == 'disturbance' else '阶跃起点'}"
              f"      : {meta['peak']:.2f} C @ {meta['t_peak']:.1f} s")
        print(f"时间常数  : tau = {fit['tau']:.2f} s  R^2 = {fit['r2']:.4f}")
        if tau_ci:
            print(f"tau 95%CI : [{tau_ci['lo']:.2f}, {tau_ci['hi']:.2f}] s"
                  f"  (u={tau_ci['std']:.2f} s, n={tau_ci['n']})")
        print(f"tau(t63)  : {t63:.2f} s")
        print(f"输出目录  : {od}")
        print(f"           report.md / curve.png / fit.json / {csv_path.name}")
    return res


def main():
    ap = argparse.ArgumentParser(description="estimate tau/theta with report + plot")
    ap.add_argument("--csv", default=None,
                    help="遥测 CSV（capture_serial.ps1 / host_logger.py 输出）")
    ap.add_argument("--selftest", action="store_true",
                    help="合成已知 τ/θ 的数据自检两条通道（不需要硬件）")
    ap.add_argument("--channel", choices=["tj", "text"], default="tj",
                    help="tj=结温通道（默认），text=传感器通道")
    ap.add_argument("--tamb", type=float, default=None,
                    help="环境温度 ℃（默认取 CSV 里 text_c 的初值）")
    ap.add_argument("--p-mw", type=float, default=None,
                    help="自热功耗 mW（默认取 CSV 的 p_mw 中位数）")
    ap.add_argument("--u-p", type=float, default=0.25, help="功耗相对不确定度（默认 0.25）")
    ap.add_argument("--u-temp", type=float, default=0.5, help="温度标准不确定度 K（默认 0.5）")
    ap.add_argument("--t0-win", type=float, default=10.0, help="起点稳态窗口 s（默认 10）")
    ap.add_argument("--boot", type=int, default=400, help="自助法重采样次数（默认 400）")
    ap.add_argument("--out", default=None,
                    help="输出目录（默认 results/<stem>_<channel>_<时间戳>/）")
    args = ap.parse_args()

    if args.selftest:
        raise SystemExit(run_selftest())
    if not args.csv:
        ap.error("需要 --csv，或用 --selftest 跑自检")

    estimate(args.csv, args.channel, p_mw_override=args.p_mw, tamb_override=args.tamb,
             u_p=args.u_p, u_temp=args.u_temp, t0_win=args.t0_win,
             n_boot=args.boot, out_dir=args.out)


if __name__ == "__main__":
    main()
