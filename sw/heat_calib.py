#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""heat_calib.py —— 加热/冷却双实验辨识 θ 与 τ（含异常值剔除、不确定度、置信区间）

用途：对应赛题「二、赛题内容-6 热阻估算」与「三、技术指标-分析要求
（热阻与热时间常数的估算须给不确定度/置信区间，并与理论量级对比）」。

方法：
    加热段（占空比 100 %，P = V²/R）：T(t) = T∞ − (T∞ − T0)·exp(−(t−t0)/τ)
        → θ = (T∞ − Tamb)/P（T∞ 由拟合外推；若已到稳态则退化为实测末值）
    冷却段（占空比 0，P = 0）：      T(t) = Tamb + (T0 − Tamb)·exp(−(t−t0)/τ)
        → 只给 τ 与 Tamb（无功率输入，不能直接得 θ）

异常值剔除：残差 MAD 迭代（|r| > max(4σ_MAD, 0.5 K) 丢弃后重拟合，最多 3 轮），
            另有物理上下限过滤（-20…120 ℃）与占空比窗口过滤。

不确定度（GUM）：
    u(θ)/θ = sqrt( (u_P/P)² + (u_T∞² + u_Tamb²)/(T∞−Tamb)² )
    u_P/P   = sqrt( (2·u_V/V)² + (u_R/R)² )        （P = V²/R）
    τ 与 T∞ 的 95 % 置信区间用残差自助法（bootstrap）。

用法：
    python sw/heat_calib.py \\
        --heat results/board_step_text_full_heating_20261122_224601/board_step.csv \\
        --cool results/board_step_text_natural_cooling_20261122_223120/board_step.csv
"""

import argparse
import csv
import json
import math
import shutil
from datetime import datetime
from pathlib import Path

import numpy as np


# ---------------------------------------------------------------------------
def load(path):
    rows = list(csv.DictReader(open(path, encoding="utf-8-sig")))
    t = np.array([float(r["t_s"]) for r in rows])
    y = np.array([float(r["text_c"]) for r in rows])
    h = np.array([int(float(r["duty_heat"])) for r in rows])
    return t, y, h


def clean(t, y, h, lo=-20.0, hi=120.0):
    """物理上下限 + 时间单调 + 去重。返回保留掩码。"""
    m = (y > lo) & (y < hi) & np.isfinite(y)
    # 时间必须严格递增（同一时刻只留第一个）
    keep = np.zeros(len(t), bool)
    last = -1e9
    for i in range(len(t)):
        if m[i] and t[i] > last:
            keep[i] = True
            last = t[i]
    return keep


def _lin_ab(e, y):
    """给定 e=exp(-(t-td)/tau)，对 y = A + B*e 做线性最小二乘。"""
    n = len(y)
    sx = e.sum(); sy = y.sum()
    sxx = (e * e).sum(); sxy = (e * y).sum()
    det = n * sxx - sx * sx
    if abs(det) < 1e-12:
        return None, None, float("inf")
    B = (n * sxy - sx * sy) / det
    A = (sy - B * sx) / n
    sse = float(np.sum((y - (A + B * e)) ** 2))
    return A, B, sse


def fit_decay(t, y, tau_range=(20.0, 900.0), n_tau=160, n_td=40, td_frac=0.20):
    """y = A + B*exp(-(t-td)/tau) 的稳健拟合。

    对 (tau, td) 做网格搜索，每个格点用线性最小二乘解 (A, B)——
    A、B 与 tau 强相关时 LM 容易跑偏（点估计甚至落在自己的 CI 外），网格法没有这个问题。
    """
    tmax = float(t[-1] - t[0])
    taus = np.geomspace(tau_range[0], tau_range[1], n_tau)
    tds = np.linspace(0.0, td_frac * tmax, n_td)
    best = None
    for td in tds:
        tt = np.clip(t - td, 0.0, None)
        for tau in taus:
            e = np.exp(-tt / tau)
            A, B, sse = _lin_ab(e, y)
            if A is None:
                continue
            if best is None or sse < best[0]:
                best = (sse, tau, td, A, B)
    # 局部细化
    _, tau0, td0, _, _ = best
    for td in np.linspace(max(0.0, td0 * 0.9 - 2), td0 * 1.1 + 2, 15):
        tt = np.clip(t - td, 0.0, None)
        for tau in np.linspace(tau0 * 0.9, tau0 * 1.1, 25):
            e = np.exp(-tt / tau)
            A, B, sse = _lin_ab(e, y)
            if A is not None and sse < best[0]:
                best = (sse, tau, td, A, B)
    sse, tau, td, A, B = best
    return dict(tau=float(tau), td=float(td), A=float(A), B=float(B),
                sse=float(sse), pred=A + B * np.exp(-np.clip(t - td, 0, None) / tau))


def robust_fit(t, y, n_iter=3, k_mad=4.0, floor=0.5):
    """迭代剔除异常值后重拟合，返回 (fit, mask, resid_std, n_out)。"""
    mask = np.ones(len(y), bool)
    fit = fit_decay(t[mask], y[mask])
    for _ in range(n_iter):
        r = y - (fit["A"] + fit["B"] * np.exp(-np.clip(t - fit["td"], 0, None) / fit["tau"]))
        med = np.median(r[mask])
        mad = np.median(np.abs(r[mask] - med)) * 1.4826
        thr = max(k_mad * mad, floor)
        new = np.abs(r - med) <= thr
        if new.sum() < 0.5 * len(y) or np.array_equal(new, mask):
            mask = new
            break
        mask = new
        fit = fit_decay(t[mask], y[mask])
    r = y[mask] - (fit["A"] + fit["B"] * np.exp(-np.clip(t[mask] - fit["td"], 0, None) / fit["tau"]))
    return fit, mask, float(np.std(r, ddof=4)), int((~mask).sum())


def bootstrap_fit(t, y, fit, n_boot=200, seed=7):
    """残差自助法：在点估计附近做局部网格，给 95% 百分位区间。"""
    pred = fit["A"] + fit["B"] * np.exp(-np.clip(t - fit["td"], 0, None) / fit["tau"])
    r = y - pred
    rng = np.random.default_rng(seed)
    taus, tds, As, Bs = [], [], [], []
    taus_g = np.linspace(fit["tau"] * 0.75, fit["tau"] * 1.3, 30)
    tds_g = np.linspace(max(0.0, fit["td"] - 15), fit["td"] + 15, 11)
    for _ in range(n_boot):
        yb = pred + rng.choice(r, size=len(r), replace=True)
        best = None
        for td in tds_g:
            tt = np.clip(t - td, 0.0, None)
            for tau in taus_g:
                A, B, sse = _lin_ab(np.exp(-tt / tau), yb)
                if A is not None and (best is None or sse < best[0]):
                    best = (sse, tau, td, A, B)
        if best:
            taus.append(best[1]); tds.append(best[2]); As.append(best[3]); Bs.append(best[4])
    a = np.array([taus, tds, As, Bs])
    lo = np.percentile(a, 2.5, axis=1)
    hi = np.percentile(a, 97.5, axis=1)
    return dict(tau=(float(lo[0]), float(hi[0])), td=(float(lo[1]), float(hi[1])),
                A=(float(lo[2]), float(hi[2])), B=(float(lo[3]), float(hi[3])),
                n=len(taus))


# ---------------------------------------------------------------------------
# 模型
def heat_model(t, Tinf, T0, tau, td):
    return Tinf - (Tinf - T0) * np.exp(-np.clip(t - td, 0, None) / tau)


def cool_model(t, Tamb, T0, tau, td):
    return Tamb + (T0 - Tamb) * np.exp(-np.clip(t - td, 0, None) / tau)


def r2(y, pred):
    ss = np.sum((y - np.mean(y)) ** 2)
    return 1.0 - np.sum((y - pred) ** 2) / ss if ss > 0 else float("nan")


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="identify theta and tau from a heat/cool pair")
    ap.add_argument("--heat", required=True)
    ap.add_argument("--cool", required=True)
    ap.add_argument("--v", type=float, default=5.0, help="加热电源电压 V（默认 5.0）")
    ap.add_argument("--r", type=float, default=10.0, help="加热电阻 Ω（默认 10）")
    ap.add_argument("--u-v", type=float, default=0.10, help="电压标准不确定度 V")
    ap.add_argument("--u-r", type=float, default=0.50, help="电阻标准不确定度 Ω")
    ap.add_argument("--u-t", type=float, default=0.5, help="传感器标准不确定度 K（AM2302）")
    ap.add_argument("--area", type=float, default=None, help="自然对流有效面积 m²（可选，做理论对比）")
    ap.add_argument("--h", type=float, default=7.5, help="自然对流传热系数 W/(m²·K)，默认 7.5")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    # ---------------- 加热段 ----------------
    th, yh, hh = load(args.heat)
    mh = clean(th, yh, hh)
    th, yh, hh = th[mh], yh[mh], hh[mh]
    on = hh >= 950                      # 满功率窗口
    i0 = int(np.argmax(on))
    t_h = th[i0:]; y_h = yh[i0:]
    t_h = t_h - t_h[0]

    fit_h, mask_h, rs_h, nout_h = robust_fit(t_h, y_h)
    Tinf, tau_h, td_h = fit_h["A"], fit_h["tau"], fit_h["td"]
    T0h = Tinf + fit_h["B"]          # B = T0 - Tinf
    pred_h = fit_h["A"] + fit_h["B"] * np.exp(-np.clip(t_h - fit_h["td"], 0, None) / fit_h["tau"])
    r2_h = r2(y_h[mask_h], pred_h[mask_h])
    bs_h = bootstrap_fit(t_h, y_h, fit_h, n_boot=200)
    hiT, loT = bs_h["A"][1], bs_h["A"][0]

    P = args.v ** 2 / args.r
    u_p_rel = math.sqrt((2 * args.u_v / args.v) ** 2 + (args.u_r / args.r) ** 2)

    # ---------------- 冷却段 ----------------
    tc, yc, hc = load(args.cool)
    mc = clean(tc, yc, hc)
    tc, yc, hc = tc[mc], yc[mc], hc[mc]
    off = hc < 50
    # 只取"从第一个关断点开始、到第一次重新加热之前"的连续段
    i0 = int(np.argmax(off))
    j = i0
    while j < len(tc) and hc[j] < 50:
        j += 1
    t_c = tc[i0:j] - tc[i0]
    y_c = yc[i0:j]

    fit_c, mask_c, rs_c, nout_c = robust_fit(t_c, y_c)
    Tamb_c, tau_c, td_c = fit_c["A"], fit_c["tau"], fit_c["td"]
    T0c = Tamb_c + fit_c["B"]
    pred_c = fit_c["A"] + fit_c["B"] * np.exp(-np.clip(t_c - fit_c["td"], 0, None) / fit_c["tau"])
    r2_c = r2(y_c[mask_c], pred_c[mask_c])
    bs_c = bootstrap_fit(t_c, y_c, fit_c, n_boot=200)

    # ---------------- θ ----------------
    u_Tinf = (hiT - loT) / 2 / 1.96
    u_Tamb = (bs_c["A"][1] - bs_c["A"][0]) / 2 / 1.96
    u_t = args.u_t
    dT = Tinf - Tamb_c
    u_dt = math.sqrt(u_Tinf ** 2 + u_Tamb ** 2 + 2 * u_t ** 2)
    theta = dT / P
    u_theta_rel = math.sqrt(u_p_rel ** 2 + (u_dt / dT) ** 2)
    theta_end = (y_h[-1] - Tamb_c) / P          # 末值（未到稳态）→ 下界

    C = tau_h / theta                            # 等效热容 J/K
    tau_rel = (bs_h["tau"][1] - bs_h["tau"][0]) / 2 / 1.96 / tau_h
    tau_c_rel = (bs_c["tau"][1] - bs_c["tau"][0]) / 2 / 1.96 / tau_c

    # ---------------- 输出 ----------------
    print("=== 加热段（P = %.2f W）===" % P)
    print("  样本 %d，剔除异常 %d 个，R² = %.4f，残差 σ = %.3f K"
          % (len(y_h), nout_h, r2_h, rs_h))
    print("  T∞ = %.2f ℃  [95%% CI %.2f, %.2f]" % (Tinf, loT, hiT))
    print("  τ  = %.1f s   [95%% CI %.1f, %.1f]" % (tau_h, *bs_h["tau"]))
    print("  时延 td = %.1f s，起点 T0 = %.2f ℃" % (td_h, T0h))
    print()
    print("=== 冷却段（P = 0）===")
    print("  样本 %d，剔除异常 %d 个，R² = %.4f，残差 σ = %.3f K"
          % (len(y_c), nout_c, r2_c, rs_c))
    print("  Tamb = %.2f ℃  [95%% CI %.2f, %.2f]" % (Tamb_c, *bs_c["A"]))
    print("  τ    = %.1f s   [95%% CI %.1f, %.1f]" % (tau_c, *bs_c["tau"]))
    print("  起点 T0 = %.2f ℃，时延 td = %.1f s" % (T0c, td_c))
    print()
    print("=== 热阻 θ = (T∞ − Tamb)/P ===")
    print("  ΔT = %.2f K，P = %.3f W" % (dT, P))
    print("  θ  = %.2f ± %.2f ℃/W  (k=1, rel %.1f %%，U95 = ±%.2f)"
          % (theta, u_theta_rel * theta, u_theta_rel * 100, 2 * u_theta_rel * theta))
    print("  分量：u(P)/P = %.1f %%，u(ΔT)/ΔT = %.1f %%" % (u_p_rel * 100, u_dt / dT * 100))
    print("  末值下界 θ_end = %.2f ℃/W（%.2f ℃，未达稳态）" % (theta_end, y_h[-1]))
    print()
    print("=== 对比 ===")
    print("  τ_heat = %.1f s  vs  τ_cool = %.1f s  → 相差 %.1f %%"
          % (tau_h, tau_c, (tau_h - tau_c) / tau_c * 100))
    print("  等效热容 C = τ/θ = %.1f J/K" % C)
    if args.area:
        theta_th = 1.0 / (args.h * args.area)
        print("  理论量级：θ = 1/(h·A) = 1/(%.1f×%.4f) = %.1f ℃/W（比值 %.2f）"
              % (args.h, args.area, theta_th, theta / theta_th))
    else:
        a_eff = 1.0 / (args.h * theta)
        print("  反推有效散热面积 A_eff = 1/(h·θ) = %.2e m² (≈%.0f cm², h=%.1f)"
              % (a_eff, a_eff * 1e4, args.h))

    out = dict(
        heating=dict(csv=str(args.heat), n=int(len(y_h)), outliers=nout_h,
                     r2=r2_h, resid_K=rs_h, T_inf=Tinf,
                     T_inf_ci=[float(loT), float(hiT)],
                     tau=tau_h, tau_ci=[float(x) for x in bs_h["tau"]],
                     td=float(td_h), T0=float(T0h), P_W=P),
        cooling=dict(csv=str(args.cool), n=int(len(y_c)), outliers=nout_c,
                     r2=r2_c, resid_K=rs_c, T_amb=Tamb_c,
                     T_amb_ci=[float(x) for x in bs_c["A"]],
                     tau=tau_c, tau_ci=[float(x) for x in bs_c["tau"]],
                     td=float(td_c), T0=float(T0c)),
        theta=dict(value=theta, u_rel=u_theta_rel, U95=2 * u_theta_rel * theta,
                   dT=float(dT), u_p_rel=u_p_rel, u_dT=float(u_dt),
                   theta_end_point=float(theta_end), C_J_per_K=float(C)),
        args=vars(args),
        generated=datetime.now().isoformat(timespec="seconds"),
    )

    od = Path(args.out) if args.out else (Path(args.heat).parent.parent /
                                          f"heatcalib_{datetime.now():%Y%m%d_%H%M%S}")
    od.mkdir(parents=True, exist_ok=True)
    (od / "heatcalib.json").write_text(json.dumps(out, ensure_ascii=False, indent=2),
                                       encoding="utf-8")
    print("\n结果 JSON ->", od / "heatcalib.json")
    return out


if __name__ == "__main__":
    main()
