#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
host_logger.py —— 串口采集 32 B 遥测帧 → CSV（分析脚本能直接吃的格式）

为什么要有这个脚本：
    上板采数据只有一次机会。如果 CSV 的列名、单位、时间基和下游脚本对不上，
    数据就白采了（返工要重新上板，而实验室时间是最贵的）。
    所以这里把"采集-解码-落盘"固定成一条命令，输出格式与 sw/analyze_thermal.py
    完全对齐：t_s / tj_c / tset_c / duty / gear / p_mw。

帧格式（对应 rtl/telemetry/frame_pack.v 的 byte_at）：
    偏移  内容
    0-1   A5 5A
    2     VER            3  TYPE
    4-5   SEQ (LE)       6-9 TS_MS (LE, 32 bit)
    10-11 TJ   Q8.8 有符号 (LE)
    12-13 TEXT Q8.8 有符号
    14-15 RH   0.1 %RH
    16-17 TSET Q8.8 有符号
    18-19 DUTY = b18 | (b19<<3)      （0–1000）
    20    GEAR          21 STATE
    22-23 F_MEAS (0.1 MHz)
    24-25 P_EST (mW)
    26    AGE(100ms)    27 FLAGS
    28-29 TSET_EXT (Q8.8)；另 b20[7:3]+b21[7:2] = DUTY_HEAT (11 bit)
    30-31 CRC16/CCITT-FALSE，覆盖字节 2–29

用法：
    :: 自检（不需要板子、不需要串口）
    python sw\\host_logger.py --selftest

    :: 采 30 min
    python sw\\host_logger.py --port COM5 --csv results\\soak_30min.csv --duration 1800

    :: 采一次阶跃（按 Ctrl+C 结束也行）
    python sw\\host_logger.py --port COM5 --csv results\\step_50C.csv ^
           --tamb 25.0 --p-mw 1200 --note "setpoint 50C, gear3"

依赖：pyserial（仅在真正连串口时需要；--selftest 不需要）
"""

import argparse
import binascii
import csv
import json
import os
import struct
import sys
import time

FRAME_LEN = 32
HDR = b"\xA5\x5A"


# ---------------------------------------------------------------------------
# 帧解码
# ---------------------------------------------------------------------------

def q8_8(lo, hi):
    """16 bit 小端 → Q8.8 有符号 → 浮点"""
    v = lo | (hi << 8)
    if v >= 0x8000:
        v -= 0x10000
    return v / 256.0


def decode_frame(b):
    """32 字节 → dict；CRC 错返回 None"""
    if len(b) != FRAME_LEN or b[0:2] != HDR:
        return None
    crc_calc = binascii.crc_hqx(bytes(b[2:30]), 0xFFFF)
    crc_recv = b[30] | (b[31] << 8)
    if crc_calc != crc_recv:
        return None

    duty = b[18] | (b[19] << 3)
    return {
        "ver":     b[2],
        "type":    b[3],
        "seq":     b[4] | (b[5] << 8),
        "ts_ms":   struct.unpack("<I", bytes(b[6:10]))[0],
        "tj_c":    q8_8(b[10], b[11]),
        "text_c":  q8_8(b[12], b[13]),
        "rh_pct":  (b[14] | (b[15] << 8)) / 10.0,
        "tset_c":  q8_8(b[16], b[17]),
        "duty":    duty,
        "gear":    b[20] & 0x07,
        "state":   b[21] & 0x03,
        "f_mhz":   (b[22] | (b[23] << 8)) / 10.0,
        "p_mw":    b[24] | (b[25] << 8),
        "age":     b[26],
        "flags":   b[27],
        "duty_heat":  ((b[20] >> 3) | ((b[21] >> 2) << 5)),
        "tset_ext_c": q8_8(b[28], b[29]),
    }


def frames_from_stream(data):
    """从字节流里切帧；容忍丢字节/半帧"""
    i = 0
    n = len(data)
    while i + FRAME_LEN <= n:
        if data[i:i + 2] != HDR:
            i += 1
            continue
        f = decode_frame(data[i:i + FRAME_LEN])
        if f is None:
            i += 1              # CRC 错 → 滑动一字节重找
            continue
        yield f
        i += FRAME_LEN


# ---------------------------------------------------------------------------
# CSV 输出
# ---------------------------------------------------------------------------

CSV_COLS = ["t_s", "tj_c", "text_c", "rh_pct", "tset_c", "duty", "gear",
            "state", "f_mhz", "p_mw", "age", "flags", "seq", "ts_ms", "tset_ext_c", "duty_heat"]


def row_of(f, t0_ms):
    return [
        round((f["ts_ms"] - t0_ms) / 1000.0, 3),
        round(f["tj_c"], 3), round(f["text_c"], 3), round(f["rh_pct"], 2),
        round(f["tset_c"], 3), f["duty"], f["gear"], f["state"],
        round(f["f_mhz"], 2), f["p_mw"], f["age"], f["flags"], f["seq"], f["ts_ms"],
    ]


def write_csv(path, rows):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as fp:
        w = csv.writer(fp)
        w.writerow(CSV_COLS)
        w.writerows(rows)


def write_meta(csv_path, meta):
    p = os.path.splitext(csv_path)[0] + ".meta.json"
    with open(p, "w", encoding="utf-8") as fp:
        json.dump(meta, fp, ensure_ascii=False, indent=2)
    return p


# ---------------------------------------------------------------------------
# 自检：造帧 → 解码 → 往返
# ---------------------------------------------------------------------------

def make_frame(seq, ts_ms, tj_q8, text_q8, rh_x10, tset_q8, duty, gear, state, f_x10, p_mw, age, flags,
               duty_heat=0, tset_ext_q8=0):
    b = bytearray(FRAME_LEN)
    b[0], b[1] = 0xA5, 0x5A
    b[2], b[3] = 0x01, 0x01
    b[4], b[5] = seq & 0xFF, (seq >> 8) & 0xFF
    b[6:10] = struct.pack("<I", ts_ms)
    b[10], b[11] = tj_q8 & 0xFF, (tj_q8 >> 8) & 0xFF
    b[12], b[13] = text_q8 & 0xFF, (text_q8 >> 8) & 0xFF
    b[14], b[15] = rh_x10 & 0xFF, (rh_x10 >> 8) & 0xFF
    b[16], b[17] = tset_q8 & 0xFF, (tset_q8 >> 8) & 0xFF
    b[18] = duty & 0x07
    b[19] = (duty >> 3) & 0xFF
    b[20] = (gear & 0x07) | ((duty_heat & 0x1F) << 3)
    b[21] = (state & 0x03) | (((duty_heat >> 5) & 0x3F) << 2)
    b[28], b[29] = tset_ext_q8 & 0xFF, (tset_ext_q8 >> 8) & 0xFF
    b[22], b[23] = f_x10 & 0xFF, (f_x10 >> 8) & 0xFF
    b[24], b[25] = p_mw & 0xFF, (p_mw >> 8) & 0xFF
    b[26], b[27] = age, flags
    crc = binascii.crc_hqx(bytes(b[2:30]), 0xFFFF)
    b[30], b[31] = crc & 0xFF, (crc >> 8) & 0xFF
    return bytes(b)


def selftest():
    ok = True

    # 1) 单帧往返
    f = make_frame(7, 123456, int(45.04 * 256), int(39.99 * 256), 551, int(50.0 * 256),
                   592, 3, 0, 1000, 1200, 2, 0x00)
    d = decode_frame(f)
    print("T1 decode      : seq=%d tj=%.2f text=%.2f rh=%.1f tset=%.1f duty=%d gear=%d f=%.1f p=%d" %
          (d["seq"], d["tj_c"], d["text_c"], d["rh_pct"], d["tset_c"], d["duty"],
           d["gear"], d["f_mhz"], d["p_mw"]))
    ok &= abs(d["tj_c"] - 45.04) < 0.01 and d["duty"] == 592 and d["seq"] == 7

    # 2) 负温度（Q8.8 补码）
    f2 = make_frame(8, 1, int(-12.5 * 256), int(-3.25 * 256), 400, int(40 * 256),
                    0, 0, 3, 250, 600, 0, 0x10)
    d2 = decode_frame(f2)
    print("T2 negative    : tj=%.2f text=%.2f (expect -12.50 / -3.25)" % (d2["tj_c"], d2["text_c"]))
    ok &= abs(d2["tj_c"] + 12.5) < 0.01 and abs(d2["text_c"] + 3.25) < 0.01

    # 3) 流里夹杂垃圾 + 半帧
    stream = b"\x00\x11\x22" + f + b"\xA5" + f2 + b"\xA5\x5A\x01"
    got = list(frames_from_stream(stream))
    print("T3 stream      : %d frames recovered (expect 2)" % len(got))
    ok &= len(got) == 2 and got[0]["seq"] == 7 and got[1]["seq"] == 8

    # 4) CRC 篡改必须被拒
    bad = bytearray(f)
    bad[10] ^= 0xFF
    print("T4 crc reject  : %s" % ("rejected" if decode_frame(bytes(bad)) is None else "NOT rejected -> BUG"))
    ok &= decode_frame(bytes(bad)) is None

    # 5) CSV 落盘 + 列名与分析脚本对齐
    rows = [row_of(g, got[0]["ts_ms"]) for g in got]
    p = os.path.join("work", "_logger_selftest.csv")
    write_csv(p, rows)
    with open(p, encoding="utf-8") as fp:
        head = fp.readline().strip()
    print("T5 csv header  : %s" % head)
    ok &= head.startswith("t_s,tj_c,tset_c") or head.startswith("t_s,tj_c,text_c")

    print("\n[%s] host_logger selftest" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# 串口采集
# ---------------------------------------------------------------------------

def run_serial(args):
    try:
        import serial
    except ImportError:
        print("缺 pyserial：pip install pyserial", file=sys.stderr)
        return 3

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    print("打开 %s @ %d，开始采集（Ctrl+C 结束）" % (args.port, args.baud))

    rows = []
    buf = bytearray()
    t_start = time.time()
    t0_ms = None
    last_seq = None
    lost = 0

    try:
        while True:
            if args.duration and (time.time() - t_start) >= args.duration:
                break
            chunk = ser.read(256)
            if chunk:
                buf.extend(chunk)
            while True:
                f = None
                for fr in frames_from_stream(bytes(buf)):
                    f = fr
                    break
                if f is None:
                    break
                idx = bytes(buf).find(HDR)
                buf = buf[idx + FRAME_LEN:]
                if t0_ms is None:
                    t0_ms = f["ts_ms"]
                if last_seq is not None and ((last_seq + 1) & 0xFFFF) != f["seq"]:
                    lost += 1
                last_seq = f["seq"]
                rows.append(row_of(f, t0_ms))
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    if not rows:
        print("一帧都没收到：检查波特率（115200）、COM 口、TX/RX 是否接反", file=sys.stderr)
        return 4

    write_csv(args.csv, rows)
    meta = {
        "csv": args.csv,
        "frames": len(rows),
        "lost_seq": lost,
        "duration_s": round(rows[-1][0], 1),
        "port": args.port, "baud": args.baud,
        "tamb_c": args.tamb, "p_mw": args.p_mw, "note": args.note,
    }
    mp = write_meta(args.csv, meta)
    print("写出 %s（%d 帧，%.1f s，丢序 %d）" % (args.csv, len(rows), rows[-1][0], lost))
    print("元数据 %s" % mp)
    return 0


def main():
    ap = argparse.ArgumentParser(description="EGO1 遥测 CSV 采集")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--csv", help="输出 CSV 路径")
    ap.add_argument("--duration", type=int, default=0, help="采集秒数，0=直到 Ctrl+C")
    ap.add_argument("--tamb", type=float, default=None, help="环境温度 ℃（写进 meta，用于 θ）")
    ap.add_argument("--p-mw", type=float, default=None, help="功率 mW（写进 meta）")
    ap.add_argument("--note", default="")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if not args.port or not args.csv:
        ap.print_help()
        return 2
    return run_serial(args)


if __name__ == "__main__":
    sys.exit(main())
