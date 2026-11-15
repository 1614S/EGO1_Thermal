#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
analyze_thermal.py —— 热响应分析（τ / θ / 阶跃指标），零第三方依赖

为什么要有这个脚本：
    初赛提交要求（数 FPGA-1）§c 明确要"基于阶跃响应的热时间常数指数拟合过程"，
    §f 明确要"θ = ΔT/P 的计算过程及不确定度来源分析"。
    这两件事必须给出可复现的算式与数字，不能只在报告里写一句话。

设计取舍：
    1. **只用标准库**。matplotlib/scipy 在别人机器上装不装得起来是不可控的，
       而 SVG 是纯文本，浏览器/VSCode/Word 都能直接看，转换也方便。
    2. **τ 用"可分离最小二乘"拟合**：固定 τ 时
           T(t) = A + B·exp(-t/τ)
       对 (A, B) 是线性的，可以解 2×2 正规方程；于是只需对 τ 做一维搜索。
       好处是不用 scipy、不会陷入局部极小、数值稳定。
    3. **置信区间用剖面似然近似**，报告里要写清这是近似值（不是 bootstrap）。

用法：
    python analyze_thermal.py --selftest                 # 自检（造已知 τ 的数据反算）
    python analyze_thermal.py --demo --out figs          # 生成演示数据 + 图
    python analyze_thermal.py --csv results/step.csv --out figs --tamb 25 --p-mw 1200

CSV 列名（大小写、单位宽松匹配；缺列不致命）：
    t_s | ts_ms | ts | time        时间（秒；若列名是 ts_ms 自动 /1000）
    tj_c | tj | temp              结温（℃）
    tset_c | tset | setpoint      设定值（℃）
    duty | duty_pm                占空比（0–1000 或 0–100）
    gear                          档位
    p_mw | p                      功率（mW）
"""

import argparse
import csv
import math
import os
import random
import sys

# ---------------------------------------------------------------------------
# CSV 读取：列名归一化，尽量容忍不同来源
# ---------------------------------------------------------------------------

def _norm(s: str) -> str:
    return "".join(ch for ch in s.strip().lower() if ch.isalnum() or ch == "_")


ALIASES = {
    "t":    ("t_s", "ts_s", "time_s", "t", "sec", "seconds"),
    "t_ms": ("ts_ms", "t_ms", "time_ms"),
    "tj":   ("tj_c", "tj", "temp_c", "temp", "temperature", "t_j_c"),
    "tset": ("tset_c", "tset", "setpoint_c", "setpoint", "target"),
    "duty": ("duty_pm", "duty", "duty_0_1000"),
    "gear": ("gear", "gear_act", "freq_gear"),
    "p":    ("p_mw", "power_mw", "p", "power"),
}


def load_csv(path):
    """读 CSV → dict of list。返回 (data, used_cols)。"""
    with open(path, "r", encoding="utf-8-sig", newline="") as f:
        rows = list(csv.reader(f))
    if not rows:
        raise ValueError("CSV 为空")

    header = [_norm(h) for h in rows[0]]
    idx = {}
    for key, names in ALIASES.items():
        for i, h in enumerate(header):
            if h in names:
                idx[key] = i
                break

    if "tj" not in idx:
        raise ValueError("CSV 里找不到结温列（tj_c / tj / temp）")
    if "t" not in idx and "t_ms" not in idx:
        raise ValueError("CSV 里找不到时间列（t_s / ts_ms）")

    data = {k: [] for k in idx}
    for r in rows[1:]:
        if not r or len(r) <= max(idx.values()):
            continue
        try:
            vals = {k: float(r[i]) for k, i in idx.items()}
        except ValueError:
            continue  # 跳过表尾注释/空行
        data.setdefault  # noqa (占位，保持可读)
        for k, v in vals.items():
            data[k].append(v)

    if "t" not in data:
        data["t"] = [x / 1000.0 for x in data["t_ms"]]
    if "duty" in data and max(data["duty"]) <= 100.0:
        data["duty"] = [x * 10.0 for x in data["duty"]]  # % → ‰

    for k in list(data.keys()):
        if not data[k]:
            del data[k]
    if len(data["t"]) != len(data["tj"]):
        raise ValueError("时间列与温度列长度不一致")
    return data


# ---------------------------------------------------------------------------
# τ 拟合：固定 τ 时对 (A, B) 线性最小二乘，对 τ 做一维搜索
# ---------------------------------------------------------------------------

def _fit_linear_given_tau(t, y, tau):
    """T(t) = A + B·exp(-t/τ) 对 A,B 的最小二乘解 → (A, B, sse)"""
    n = len(t)
    e = [math.exp(-ti / tau) for ti in t]
    s1 = float(n)
    se = sum(e)
    see = sum(x * x for x in e)
    sy = sum(y)
    sey = sum(ei * yi for ei, yi in zip(e, y))

    det = s1 * see - se * se
    if abs(det) < 1e-12:
        return None
    a = (sy * see - se * sey) / det
    b = (s1 * sey - se * sy) / det
    sse = sum((yi - (a + b * ei)) ** 2 for yi, ei in zip(y, e))
    return a, b, sse


def fit_tau(t, y, tau_lo=0.1, tau_hi=600.0, samples=400):
    """一维黄金分割搜索 τ，返回 dict(tau, tau_ci, r2, A, B, n)。"""
    n = len(t)
    if n < 5:
        raise ValueError("样本太少，无法拟合 τ")

    def sse_of(tau):
        r = _fit_linear_given_tau(t, y, tau)
        return float("inf") if r is None else r[2]

    # 先在 log 轴上粗扫，避免黄金分割落进错误的区间
    best_tau, best_sse = tau_lo, float("inf")
    for i in range(samples):
        tau = tau_lo * (tau_hi / tau_lo) ** (i / (samples - 1))
        s = sse_of(tau)
        if s < best_sse:
            best_sse, best_tau = s, tau

    # 在最优附近做黄金分割细化
    lo, hi = best_tau / 1.5, best_tau * 1.5
    gr = (math.sqrt(5.0) - 1.0) / 2.0
    c = hi - gr * (hi - lo)
    d = lo + gr * (hi - lo)
    fc, fd = sse_of(c), sse_of(d)
    for _ in range(80):
        if fc < fd:
            hi, d, fd = d, c, fc
            c = hi - gr * (hi - lo)
            fc = sse_of(c)
        else:
            lo, c, fc = c, d, fd
            d = lo + gr * (hi - lo)
            fd = sse_of(d)
    tau = (lo + hi) / 2.0
    a, b, sse = _fit_linear_given_tau(t, y, tau)

    ybar = sum(y) / n
    sst = sum((yi - ybar) ** 2 for yi in y)
    r2 = 1.0 - sse / sst if sst > 0 else 0.0

    # 剖面似然近似 95% 区间：SSE(τ) ≤ SSE_min·(1 + F/n)
    # F≈4 对应 3 参数、大样本的 95% 量级，仅作量级参考（报告里写明是近似）
    sse_at = lambda tau_x: sse_of(tau_x)  # noqa: E731
    target = sse * (1.0 + 4.0 / n)

    def _edge(direction):
        """从最优点向外搜，步长先细后粗（避免干净数据下区间退化成一点）"""
        cur = tau
        step = max(1e-6, tau * 1e-4)
        for _ in range(400):
            nxt = cur + direction * step
            if nxt <= tau_lo * 0.5 or nxt >= tau_hi * 2.0:
                return cur
            if sse_at(nxt) > target:
                return nxt
            cur = nxt
            step *= 1.08
        return cur

    lo_ci = _edge(-1.0)
    hi_ci = _edge(+1.0)

    return {
        "tau": tau, "tau_lo": lo_ci, "tau_hi": hi_ci,
        "A": a, "B": b, "r2": r2, "n": n, "sse": sse,
    }


# ---------------------------------------------------------------------------
# 阶跃指标：超调 / 调节时间 / 稳态误差
# ---------------------------------------------------------------------------

def step_metrics(t, y, tset, band=1.0, settle_hold=5.0, ess_window=10.0):
    """以 tset 为终值，返回 overshoot(%) / settle_time(s) / ess(℃)"""
    y_end = sum(y[-max(1, len(y) // 10):]) / max(1, len(y) // 10)
    y0 = y[0]
    delta = tset - y0
    over = 0.0
    if abs(delta) > 1e-9:
        over = max(0.0, (max(y) - tset) / delta * 100.0) if delta > 0 else 0.0

    settle = float("nan")
    for i in range(len(t)):
        if all(abs(v - tset) <= band for v in y[i:]):
            settle = t[i] - t[0]
            break

    t_last = t[-1]
    tail = [v for ti, v in zip(t, y) if ti >= t_last - ess_window]
    ess = abs(sum(tail) / len(tail) - tset) if tail else float("nan")
    return {"overshoot_pct": over, "settle_s": settle, "ess_c": ess,
            "y_end": y_end, "delta": delta}


# ---------------------------------------------------------------------------
# 热阻 θ 与不确定度
# ---------------------------------------------------------------------------

def theta_with_uncertainty(tj_ss, t_amb, p_w, u_tj=0.5, u_tamb=0.5, u_p_rel=0.25):
    """
    θ = (Tj - Tamb) / P
    u(θ)/θ = sqrt( (u(ΔT)/ΔT)^2 + (u(P)/P)^2 )，u(ΔT) = sqrt(u_tj^2 + u_tamb^2)

    返回 dict(theta, u_dt, u_theta, rel)  ← 单位 ℃/W
    """
    dt = tj_ss - t_amb
    if p_w <= 0:
        raise ValueError("功率必须为正")
    if abs(dt) < 1e-9:
        raise ValueError("ΔT 太小，无法估算热阻")

    u_dt = math.sqrt(u_tj ** 2 + u_tamb ** 2)
    theta = dt / p_w
    rel = math.sqrt((u_dt / dt) ** 2 + u_p_rel ** 2)
    return {"theta": theta, "u_theta": theta * rel, "rel": rel,
            "dt": dt, "u_dt": u_dt, "p_w": p_w}


# ---------------------------------------------------------------------------
# SVG 绘图（纯文本，零依赖）
# ---------------------------------------------------------------------------

def write_svg(path, t, y, tset=None, duty=None, gear=None, title="thermal step response"):
    """把温度（以及可选的占空比/档位）画成 SVG。"""
    W, H = 960, 420
    ML, MR, MT, MB = 70, 30, 40, 50
    pw, ph = W - ML - MR, H - MT - MB

    ymin = min(min(y), tset if tset is not None else min(y))
    ymax = max(max(y), tset if tset is not None else max(y))
    pad = max(1.0, (ymax - ymin) * 0.1)
    ymin -= pad
    ymax += pad
    t0, t1 = t[0], t[-1]
    dt = (t1 - t0) or 1.0

    def sx(tv):
        return ML + (tv - t0) / dt * pw

    def sy(v):
        return MT + (ymax - v) / (ymax - ymin) * ph

    parts = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" font-family="Consolas,monospace">' % (W, H),
        '<rect width="100%%" height="100%%" fill="#ffffff"/>',
        '<text x="%d" y="22" font-size="15" fill="#222">%s</text>' % (ML, _esc(title)),
    ]

    # 坐标轴与网格
    for k in range(5):
        v = ymin + (ymax - ymin) * k / 4.0
        yy = sy(v)
        parts.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" stroke="#eee"/>' % (ML, yy, ML + pw, yy))
        parts.append('<text x="%d" y="%.1f" font-size="11" fill="#666" text-anchor="end">%.1f</text>'
                     % (ML - 8, yy + 4, v))
    parts.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#888"/>' % (ML, MT + ph, ML + pw, MT + ph))
    parts.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#888"/>' % (ML, MT, ML, MT + ph))

    for k in range(6):
        tv = t0 + dt * k / 5.0
        parts.append('<text x="%.1f" y="%d" font-size="11" fill="#666" text-anchor="middle">%.0f</text>'
                     % (sx(tv), MT + ph + 18, tv))
    parts.append('<text x="%d" y="%d" font-size="12" fill="#444" text-anchor="middle">t (s)</text>'
                 % (ML + pw // 2, H - 10))
    parts.append('<text x="18" y="%d" font-size="12" fill="#444" transform="rotate(-90 18 %d)" text-anchor="middle">T (degC)</text>'
                 % (MT + ph // 2, MT + ph // 2))

    # 设定值
    if tset is not None:
        yy = sy(tset)
        parts.append('<line x1="%d" y1="%.1f" x2="%d" y2="%.1f" stroke="#d33" stroke-dasharray="6,4"/>'
                     % (ML, yy, ML + pw, yy))
        parts.append('<text x="%d" y="%.1f" font-size="11" fill="#d33">setpoint %.1f</text>'
                     % (ML + pw - 110, yy - 6, tset))

    # 温度曲线
    pts = " ".join("%.1f,%.1f" % (sx(ti), sy(yi)) for ti, yi in zip(t, y))
    parts.append('<polyline fill="none" stroke="#06c" stroke-width="1.6" points="%s"/>' % pts)

    # 占空比（右轴，画成灰线，只作参考）
    if duty:
        dmax = max(duty) or 1
        pts = " ".join("%.1f,%.1f" % (sx(ti), MT + ph - (di / dmax) * ph * 0.35)
                       for ti, di in zip(t, duty))
        parts.append('<polyline fill="none" stroke="#9a9" stroke-width="1" stroke-dasharray="3,3" points="%s"/>' % pts)
        parts.append('<text x="%d" y="%d" font-size="11" fill="#797">duty (dashed)</text>'
                     % (ML + pw - 110, MT + 14))

    parts.append("</svg>")
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))
    return path


def _esc(s):
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


# ---------------------------------------------------------------------------
# 自检：造已知 τ 的数据反算
# ---------------------------------------------------------------------------

def selftest():
    ok = True
    rnd = random.Random(42)

    # --- 1) τ 拟合 ---
    tau_true, t_inf, y0 = 18.0, 50.0, 30.0
    t = [0.5 * i for i in range(400)]
    y = [t_inf - (t_inf - y0) * math.exp(-ti / tau_true) + rnd.gauss(0, 0.05) for ti in t]
    r = fit_tau(t, y)
    err = abs(r["tau"] - tau_true) / tau_true * 100
    print("T1 tau fit       : true %.2f s  ->  fitted %.2f s (%.2f%% err, R2=%.4f)" %
          (tau_true, r["tau"], err, r["r2"]))
    ok &= err < 3.0 and r["r2"] > 0.99

    # --- 2) 含初值偏移的稳健性（模拟采样前已过了一段时间）---
    y2 = [t_inf - (t_inf - y0) * math.exp(-(ti + 3.0) / tau_true) for ti in t]
    r2 = fit_tau(t, y2)
    err2 = abs(r2["tau"] - tau_true) / tau_true * 100
    print("T2 tau offset    : fitted %.2f s (%.2f%% err)" % (r2["tau"], err2))
    ok &= err2 < 3.0

    # --- 3) 阶跃指标 ---
    tset = 50.0
    m = step_metrics(t, y, tset)
    print("T3 step metrics  : overshoot %.1f%%  settle %.1f s  ess %.2f degC" %
          (m["overshoot_pct"], m["settle_s"], m["ess_c"]))
    ok &= abs(m["ess_c"]) < 0.3 and math.isfinite(m["settle_s"])

    # --- 4) θ 与不确定度 ---
    th = theta_with_uncertainty(tj_ss=50.0, t_amb=25.0, p_w=1.2, u_tj=0.5, u_tamb=0.5, u_p_rel=0.25)
    print("T4 theta         : %.2f +/- %.2f degC/W (rel %.1f%%)" %
          (th["theta"], th["u_theta"], th["rel"] * 100))
    ok &= abs(th["theta"] - 20.833) < 0.01
    ok &= abs(th["rel"] - math.sqrt((0.7071 / 25.0) ** 2 + 0.25 ** 2)) < 1e-6

    # --- 5) CSV 往返 ---
    p = os.path.join("work", "_selftest.csv")
    os.makedirs("work", exist_ok=True)
    with open(p, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "tj_c", "tset_c", "duty", "gear"])
        for ti, yi in zip(t, y):
            w.writerow(["%.2f" % ti, "%.3f" % yi, "%.1f" % tset, "500", "3"])
    d = load_csv(p)
    print("T5 csv roundtrip : %d rows, t[0]=%.2f, tj[0]=%.2f" % (len(d["t"]), d["t"][0], d["tj"][0]))
    ok &= len(d["t"]) == len(t)

    # --- 6) SVG 能不能生成 ---
    svg = write_svg(os.path.join("work", "_selftest.svg"), t, y, tset=tset)
    print("T6 svg           : %s (%d bytes)" % (svg, os.path.getsize(svg)))
    ok &= os.path.getsize(svg) > 1000

    print("\n[%s] analyze_thermal selftest" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# 演示 / 主流程
# ---------------------------------------------------------------------------

def make_demo(out_dir):
    """造一段"阶跃 + 一阶热响应 + 噪声"的数据，用来自检图与流程（不是实测数据）"""
    os.makedirs(out_dir, exist_ok=True)
    tau, t_inf, y0, tset = 45.0, 50.0, 32.0, 50.0
    rnd = random.Random(7)
    t, y, duty = [], [], []
    for i in range(1800):                    # 0.5 s × 1800 = 900 s
        ti = 0.5 * (i + 1)
        v = t_inf - (t_inf - y0) * math.exp(-ti / tau) + rnd.gauss(0, 0.06)
        t.append(ti)
        y.append(v)
        duty.append(min(1000, max(0, 600 * (1 - math.exp(-ti / tau)) + rnd.gauss(0, 8))))
    path = os.path.join(out_dir, "demo_step.csv")
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "tj_c", "tset_c", "duty", "gear", "p_mw"])
        for ti, yi, di in zip(t, y, duty):
            w.writerow(["%.2f" % ti, "%.3f" % yi, "%.1f" % tset, "%d" % int(di), "3", "1200"])
    return path, t, y, tset, duty


def main():
    ap = argparse.ArgumentParser(description="thermal response analysis (tau / theta / step)")
    ap.add_argument("--csv", help="输入 CSV")
    ap.add_argument("--out", default="figs", help="输出目录（SVG）")
    ap.add_argument("--tamb", type=float, default=None, help="环境温度 ℃（给了就顺便算 θ）")
    ap.add_argument("--p-mw", type=float, default=None, help="功率 mW（给了就顺便算 θ）")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--demo", action="store_true", help="造演示数据并出图")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    if args.demo:
        path, t, y, tset, duty = make_demo(args.out)
        print("demo csv -> %s" % path)
    elif args.csv:
        d = load_csv(args.csv)
        t, y = d["t"], d["tj"]
        tset = (sum(d["tset"]) / len(d["tset"])) if "tset" in d else None
        duty = d.get("duty")
        path = args.csv
    else:
        ap.print_help()
        return 2

    r = fit_tau(t, y)
    print("\n== tau ==")
    print("  tau    = %.2f s  (95%% approx [%.2f, %.2f])" % (r["tau"], r["tau_lo"], r["tau_hi"]))
    print("  R^2    = %.4f   n = %d" % (r["r2"], r["n"]))

    if tset is not None:
        m = step_metrics(t, y, tset)
        print("\n== step ==")
        print("  overshoot = %.1f %%" % m["overshoot_pct"])
        print("  settle    = %.1f s (band +/-1.0 degC, hold 5 s)" % m["settle_s"])
        print("  ess       = %.2f degC" % m["ess_c"])

    if args.tamb is not None and args.p_mw:
        th = theta_with_uncertainty(y[-1], args.tamb, args.p_mw / 1000.0)
        print("\n== theta ==")
        print("  dT     = %.2f degC, P = %.3f W" % (th["dt"], th["p_w"]))
        print("  theta  = %.2f +/- %.2f degC/W  (k=1, rel %.1f %%)" %
              (th["theta"], th["u_theta"], th["rel"] * 100))

    svg = write_svg(os.path.join(args.out, "thermal_step.svg"), t, y, tset=tset, duty=duty)
    print("\nplot -> %s" % svg)
    return 0


if __name__ == "__main__":
    sys.exit(main())
