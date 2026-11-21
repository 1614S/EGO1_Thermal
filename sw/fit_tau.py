#!/usr/bin/env python3
"""fit_tau.py —— 从遥测 CSV 估算热时间常数 tau 与热阻 theta（板级实测）

与 analyze_thermal.py 的区别：
  * 本脚本面向"外部热扰动"实验（把板子靠近热源再移开），自动找峰值后拟合**降温段**；
  * theta 用**起点稳态**算：theta = (T0 - Tamb) / P，T0 取前 100 帧均值；
  * Tamb 默认取 CSV 里 AM2302 的 text_c（实测），可用 --tamb 覆盖；
  * P 默认取 CSV 的 p_mw（实测功耗），可用 --p-mw 覆盖。

用法：
  python sw/fit_tau.py --csv results/tau_theta_run.csv
  python sw/fit_tau.py --csv results/x.csv --tamb 23.0 --p-mw 750

输出：
  T0 / Tamb / P / theta / 峰值 / tau(指数拟合, R^2) / tau(t63)
"""

import argparse
import csv
import math


def load(path):
    # capture_serial.ps1 写的是带 BOM 的 UTF-8，必须用 utf-8-sig
    with open(path, encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"CSV 是空的：{path}")
    t = [float(r["t_s"]) for r in rows]
    y = [float(r["tj_c"]) for r in rows]
    return rows, t, y


def fit_tau(t2, y2, lo=0.2, hi=600.0, step=1.01):
    """给定 tau 时对 A、B 做线性最小二乘，网格搜索使 SSE 最小的 tau。"""
    best = None
    tau = lo
    while tau < hi:
        e = [math.exp(-ti / tau) for ti in t2]
        n = len(t2)
        sx = sum(e)
        sy = sum(y2)
        sxx = sum(v * v for v in e)
        sxy = sum(a * b for a, b in zip(e, y2))
        det = n * sxx - sx * sx
        if abs(det) > 1e-12:
            B = (n * sxy - sx * sy) / det
            A = (sy - B * sx) / n
            sse = sum((yy - (A + B * ee)) ** 2 for yy, ee in zip(y2, e))
            if best is None or sse < best[0]:
                best = (sse, tau, A, B)
        tau *= step
    if best is None:
        raise SystemExit("拟合失败：数据点太少或全是常数")
    sse, tau, A, B = best
    sst = sum((yy - sum(y2) / len(y2)) ** 2 for yy in y2)
    r2 = 1.0 - sse / sst if sst > 0 else float("nan")
    return tau, A, B, r2


def main():
    ap = argparse.ArgumentParser(description="estimate tau / theta from a telemetry CSV")
    ap.add_argument("--csv", required=True, help="capture_serial.ps1 输出的 CSV")
    ap.add_argument("--tamb", type=float, default=None, help="环境温度 ℃（默认取 CSV 的 text_c）")
    ap.add_argument("--p-mw", type=float, default=None, help="自热功耗 mW（默认取 CSV 的 p_mw）")
    args = ap.parse_args()

    rows, t, y = load(args.csv)
    tamb = args.tamb if args.tamb is not None else float(rows[-1]["text_c"])
    p_w = (args.p_mw if args.p_mw is not None else float(rows[-1]["p_mw"])) / 1000.0

    # 起点稳态（前 100 帧 ≈ 10 s）
    n0 = min(100, len(y))
    t0 = sum(y[:n0]) / n0
    theta = (t0 - tamb) / p_w
    u_dt = math.sqrt(0.5 ** 2 + 0.5 ** 2)
    u_rel = math.sqrt((u_dt / (t0 - tamb)) ** 2 + 0.25 ** 2)

    # 找峰值，拟合峰后降温段
    ip = max(range(len(y)), key=lambda i: y[i])
    t2 = [ti - t[ip] for ti in t[ip:]]
    y2 = y[ip:]
    y_inf = sum(y2[-min(100, len(y2)):]) / min(100, len(y2))

    tau, A, B, r2 = fit_tau(t2, y2)
    target = y_inf + (y2[0] - y_inf) * math.exp(-1)
    t63 = next((ti for ti, yy in zip(t2, y2) if yy <= target), float("nan"))

    print(f"file      : {args.csv}  ({len(y)} 帧)")
    print(f"起点稳态  : T0 = {t0:.2f} ℃")
    print(f"环境温度  : Tamb = {tamb:.2f} ℃")
    print(f"自热功耗  : P = {p_w * 1000:.0f} mW")
    print(f"theta     : {theta:.2f} ± {theta * u_rel:.2f} ℃/W  (k=1, rel {u_rel * 100:.1f} %)")
    print(f"峰值      : {max(y):.2f} ℃ @ t = {t[ip]:.1f} s，回落稳态 {y_inf:.2f} ℃")
    print(f"tau(拟合) : {tau:.2f} s   R^2 = {r2:.4f}   A = {A:.2f}  B = {B:.2f}")
    print(f"tau(t63)  : {t63:.2f} s")


if __name__ == "__main__":
    main()
