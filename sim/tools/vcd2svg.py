#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
vcd2svg.py —— 把仿真 VCD 渲染成波形图（SVG，零第三方依赖）

为什么不用 GTKWave 截图 / matplotlib：
    1. 截图不可复现、不可批量，报告要 4～6 张图时会很痛苦；
    2. matplotlib 装不装得起来在别人机器上不可控，而 SVG 是纯文本。
    本脚本只解析 VCD 文本，输出 SVG，任何机器上 `python vcd2svg.py` 就能跑。

用法：
    python vcd2svg.py --vcd wave.vcd --out out.svg ^
        --signal "gear_req:digital:gearbit" ^
        --signal "tj_q8_8:analog:结温/256"

    --signal 格式： 名字子串:类型[:标签]
        类型 = digital（数字波形，可多位则画成总线块）
             | analog （把多位值当数值画成折线，适合温度/占空比）

    --time-start / --time-end  只画某个时间窗（单位与被选信号一致，默认 ns）

VCD 来源（用 xsim 对现有 TB 抓，不需要改 TB）：
    xsim <snapshot> -R -tclbatch dump.tcl
    dump.tcl:
        open_vcd wave.vcd
        log_vcd [get_objects /tb_top_smoke/dut/*]
        run all
        close_vcd

约定：只支持 VCD 的标量/向量（b/r 变化），够用；不支持 $dumpvars 的压缩格式扩展。
"""

import argparse
import math
import os
import re
import sys

# ---------------------------------------------------------------------------
# VCD 解析
# ---------------------------------------------------------------------------

class Vcd:
    def __init__(self):
        self.ids = {}        # id -> (full_name, width, vartype)
        self.times = []      # 每个变化时刻
        self.changes = {}    # id -> [(t, value_str), ...]
        self.end_time = 0
        self.time_unit_ns = 1.0   # VCD 的时间单位（# 后面的数字）换算到 ns

    def parse(self, path):
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            lines = f.read().splitlines()

        scope = []
        i = 0
        n = len(lines)
        cur_t = 0

        while i < n:
            ln = lines[i].strip()
            i += 1
            if not ln:
                continue

            if ln.startswith("$scope"):
                parts = ln.split()
                scope.append(parts[2] if len(parts) > 2 else "?")
            elif ln.startswith("$upscope"):
                if scope:
                    scope.pop()
            elif ln.startswith("$var"):
                # $var wire 16 ! tj [15:0] $end
                parts = ln.split()
                if len(parts) >= 5:
                    vtype, width, vid, name = parts[1], int(parts[2]), parts[3], parts[4]
                    full = ".".join(scope + [name])
                    self.ids[vid] = (full, width, vtype)
                    self.changes.setdefault(vid, [])
            elif ln.startswith("$enddefinitions"):
                continue
            elif ln.startswith("$timescale"):
                # 可能写成 "$timescale 1ns $end"，也可能单位在下一行
                parts = ln.split()
                if len(parts) >= 2 and parts[1] != "$end":
                    self.time_unit_ns = _unit_to_ns(parts[1])
                elif i < n:
                    self.time_unit_ns = _unit_to_ns(lines[i].strip())
                    i += 1
            elif ln.startswith("$dumpvars") or ln.startswith("$end") \
                    or ln.startswith("$comment") or ln.startswith("$version") \
                    or ln.startswith("$date"):
                continue
            elif ln[0] == "#":
                try:
                    cur_t = int(ln[1:])
                except ValueError:
                    pass
                self.end_time = max(self.end_time, cur_t)
            elif ln[0] in "01xzXZ":
                # 标量：0!
                if len(ln) >= 2:
                    vid = ln[1:]
                    if vid in self.changes:
                        self.changes[vid].append((cur_t, ln[0]))
            elif ln[0] in "bB":
                # 向量： b1010 !
                parts = ln.split()
                if len(parts) >= 2:
                    bits, vid = parts[0][1:], parts[1]
                    if vid in self.changes:
                        self.changes[vid].append((cur_t, bits))
            elif ln[0] in "rR":
                parts = ln.split()
                if len(parts) >= 2 and parts[1] in self.changes:
                    self.changes[parts[1]].append((cur_t, parts[0][1:]))
            # 其余（$dumpports 等）忽略

        # 统一换算成 ns：VCD 的 # 数字用 $timescale 单位（Icarus 默认 1ps）
        if self.time_unit_ns != 1.0:
            f = self.time_unit_ns
            for k in self.changes:
                self.changes[k] = [(int(round(t * f)), v) for (t, v) in self.changes[k]]
            self.end_time = int(round(self.end_time * f))
        return self

    def find(self, substr):
        """按名字子串找信号，返回 [(full, width, id)]。优先'最短名字'最匹配的。"""
        hits = []
        for vid, (full, width, vtype) in self.ids.items():
            if substr in full:
                hits.append((full, width, vid))
        hits.sort(key=lambda x: (len(x[0]), x[0]))
        return hits


def _unit_to_ns(s):
    """把 "1ps" / "10ns" / "1us" 这类 VCD 时间单位换算成 ns"""
    m = re.match(r"\s*(\d+(?:\.\d+)?)\s*(fs|ps|ns|us|ms|s)\b", s)
    if not m:
        return 1.0
    num = float(m.group(1))
    mult = {"fs": 1e-6, "ps": 1e-3, "ns": 1.0, "us": 1e3, "ms": 1e6, "s": 1e9}[m.group(2)]
    return num * mult


def _to_num(v, width):
    """把 VCD 值字符串转成数值；x/z 位按 0 处理（并在调用处标注）"""
    if not v:
        return None
    if v[0] in "xXzZ":
        return None
    try:
        return int(v, 2)
    except ValueError:
        return None


# ---------------------------------------------------------------------------
# SVG 渲染
# ---------------------------------------------------------------------------

COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e", "#8c564b"]


def render(vcd, picks, out_path, t_start=None, t_end=None, title="", scale=1.0):
    """
    picks: [ (label, kind, id, width) ]
    纵向每行一个信号：digital 画方波（多位按值段画块），analog 画折线。
    """
    t0 = t_start if t_start is not None else 0
    t1 = t_end if t_end is not None else vcd.end_time
    if t1 <= t0:
        raise ValueError("时间窗为空")

    ROW_H = 54
    W = int(1180 * scale)
    LEFT, RIGHT, TOP, BOT = 210, 40, 46, 46
    plot_w = W - LEFT - RIGHT
    H = int(TOP + BOT + ROW_H * len(picks))

    def sx(t):
        return LEFT + (t - t0) / float(t1 - t0) * plot_w

    parts = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" '
        'font-family="Consolas,Menlo,monospace" font-size="11">' % (W, H),
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        '<text x="%d" y="24" font-size="14" fill="#111">%s</text>' % (LEFT, _esc(title)),
    ]

    # 时间刻度
    for k in range(7):
        t = t0 + (t1 - t0) * k / 6.0
        x = sx(t)
        parts.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="#eee"/>'
                     % (x, TOP - 6, x, H - BOT + 4))
        parts.append('<text x="%.1f" y="%d" fill="#666" text-anchor="middle">%s</text>'
                     % (x, H - BOT + 18, _fmt_time(t)))
    parts.append('<text x="%d" y="%d" fill="#444" text-anchor="middle">%s</text>'
                 % (LEFT + plot_w // 2, H - 10, _esc("time")))

    for idx, (label, kind, vid, width) in enumerate(picks):
        ytop = TOP + idx * ROW_H
        ymid = ytop + ROW_H // 2
        parts.append('<text x="%d" y="%d" fill="#111" text-anchor="end">%s</text>'
                     % (LEFT - 10, ymid + 4, _esc(label)))

        evs = [(t, v) for (t, v) in vcd.changes.get(vid, []) if t0 - 1 <= t <= t1]
        if not evs:
            parts.append('<text x="%d" y="%d" fill="#c00">(无变化)</text>' % (LEFT + 6, ymid + 4))
            continue

        # 补一个窗口起点事件，避免折线/方波从空白开始
        if evs[0][0] > t0:
            evs.insert(0, (t0, evs[0][1]))

        if kind == "analog" or (width > 1 and kind == "auto"):
            parts.extend(_draw_analog(evs, sx, ytop, ROW_H, width, COLORS[idx % len(COLORS)], LEFT, t1))
        else:
            parts.extend(_draw_digital(evs, sx, ytop, ROW_H, width, COLORS[idx % len(COLORS)], t1))

    parts.append("</svg>")
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(parts))
    return out_path


def _draw_digital(evs, sx, ytop, rh, width, color, t_end):
    out = []
    yhi, ylo = ytop + 12, ytop + rh - 12
    prev_t = None
    prev_v = None
    for (t, v) in evs:
        num = _to_num(v, width)
        if num is None:
            continue
        y = yhi if num != 0 else ylo
        if prev_t is not None:
            out.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="%s" stroke-width="1.4"/>'
                       % (sx(prev_t), prev_v, sx(t), prev_v, color))
            if y != prev_v:
                out.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="%s" stroke-width="1.4"/>'
                           % (sx(t), prev_v, sx(t), y, color))
        prev_t, prev_v = t, y
    if prev_t is not None:
        out.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="%s" stroke-width="1.4"/>'
                   % (sx(prev_t), prev_v, sx(prev_t) + 1, prev_v, color))
    return out


def _draw_analog(evs, sx, ytop, rh, width, color, left, t_end):
    ys = []
    pts = []
    for (t, v) in evs:
        num = _to_num(v, width)
        if num is None:
            continue
        # 有符号解释（Q8.8 会是负数）
        if num >= (1 << (width - 1)):
            num -= (1 << width)
        pts.append((t, num))
        ys.append(num)
    if not pts:
        return []
    ymin, ymax = min(ys), max(ys)
    if ymax == ymin:
        ymax = ymin + 1
    pad = (ymax - ymin) * 0.12
    ymin -= pad
    ymax += pad

    def sy(v):
        return ytop + rh - 12 - (v - ymin) / (ymax - ymin) * (rh - 24)

    pts.append((t_end, pts[-1][1]))   # 末尾无变化时把线画到窗口右边界
    d = " ".join("%.1f,%.1f" % (sx(t), sy(v)) for t, v in pts)
    out = ['<polyline fill="none" stroke="%s" stroke-width="1.6" points="%s"/>' % (color, d)]
    out.append('<text x="%d" y="%d" fill="#888" font-size="10" text-anchor="end">%d</text>'
               % (left - 6, ytop + 14, int(max(ys))))
    out.append('<text x="%d" y="%d" fill="#888" font-size="10" text-anchor="end">%d</text>'
               % (left - 6, ytop + rh - 4, int(min(ys))))
    return out


def _fmt_time(t):
    if t >= 1e9:
        return "%.1f s" % (t / 1e9)
    if t >= 1e6:
        return "%.1f ms" % (t / 1e6)
    if t >= 1e3:
        return "%.0f us" % (t / 1e3)
    return "%d" % t


def _esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vcd", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--signal", action="append", default=[],
                    help="名字子串:digital|analog[:标签]，可重复")
    ap.add_argument("--t-start", type=float, default=None)
    ap.add_argument("--t-end", type=float, default=None)
    ap.add_argument("--title", default="")
    ap.add_argument("--list", action="store_true", help="列出 VCD 里的所有信号")
    args = ap.parse_args()

    v = Vcd().parse(args.vcd)
    print("VCD: %s  (%d 个信号, 结束时间 %d)" % (args.vcd, len(v.ids), v.end_time))

    if args.list:
        for vid, (full, width, vt) in sorted(v.ids.items(), key=lambda x: x[1][0]):
            print("  %-60s width=%d" % (full, width))
        return 0

    picks = []
    for spec in args.signal:
        bits = spec.split(":")
        name = bits[0]
        kind = bits[1] if len(bits) > 1 else "digital"
        label = bits[2] if len(bits) > 2 else name
        hits = v.find(name)
        if not hits:
            print("  [WARN] 找不到信号: %s" % name, file=sys.stderr)
            continue
        full, width, vid = hits[0]
        picks.append((label, kind, vid, width))
        print("  pick: %-58s width=%d  <- %s" % (full, width, label))

    if not picks:
        print("没有任何信号被选中", file=sys.stderr)
        return 2

    out = render(v, picks, args.out, args.t_start, args.t_end, args.title or os.path.basename(args.vcd))
    print("SVG -> %s (%d bytes)" % (out, os.path.getsize(out)))
    return 0


if __name__ == "__main__":
    sys.exit(main())

