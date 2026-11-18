#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""把 tb_closed_loop 的三档轨迹按设定值变化拆成三段 CSV。

analyze_thermal.py 假设"一段数据 = 一次阶跃"，混着三段拟合 τ 会得到 R²≈0.09
（实测），所以必须先拆开。
"""
import csv
import pathlib
import sys

src = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else r"D:\.EGO1\.work\closed_loop_trace.csv")
outdir = pathlib.Path(sys.argv[2] if len(sys.argv) > 2 else r"D:\.EGO1\.work")

rows = list(csv.DictReader(src.open(encoding="utf-8")))
if not rows:
    print("空 CSV")
    sys.exit(1)

phases = []
cur = [rows[0]]
for r in rows[1:]:
    if r["tset_c"] != cur[-1]["tset_c"]:
        phases.append(cur)
        cur = [r]
    else:
        cur.append(r)
phases.append(cur)

print(f"共 {len(rows)} 行，拆成 {len(phases)} 段：")
for i, ph in enumerate(phases, 1):
    sp = float(ph[0]["tset_c"])
    name = outdir / f"phase{i}_{sp:.0f}C.csv"
    with name.open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "tj_c", "tset_c", "duty", "gear", "p_mw"])
        t0 = float(ph[0]["t_s"])
        for r in ph:
            # 时间从 0 开始，方便拟合
            w.writerow([f'{float(r["t_s"]) - t0:.2f}', r["tj_c"], r["tset_c"],
                        r["duty"], r["gear"], r["p_mw"]])
    print(f"  {name.name}: {len(ph)} 行，Tset={sp:.1f}℃，{ph[0]['tj_c']}→{ph[-1]['tj_c']}℃")
