#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""交叉验证：把 RTL 产生的 32 B 帧交给 sw/host_logger.py 解码。

两套独立实现（Verilog 的 frame_pack 与 Python 的 host_logger）对同一帧
必须解出同样的字段 —— 这是"上位机脚本能直接用"的最直接证据。

帧由 tb_frame 打印（FRAME_HEX=...），字段由 TB 的激励决定，见下表。
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "sw"))
import host_logger as HL  # noqa: E402

# tb_frame 激励：seq=0x1234, ts=0x00010203, tj=12800(50.0C), text=6400(25.0C),
#               rh=550, tset=12800, duty=500, gear=3, state=0, f=1000, p=1200
HEX = "a55a01013412030201000032001926020032043e0300e803b00400000000a810"

EXPECT = {
    "seq": 0x1234, "ts_ms": 0x00010203,
    "tj_c": 50.0, "text_c": 25.0, "rh_pct": 55.0, "tset_c": 50.0,
    "duty": 500, "gear": 3, "state": 0, "f_mhz": 100.0, "p_mw": 1200,
}


def main():
    b = bytes.fromhex(HEX)
    f = HL.decode_frame(b)
    if f is None:
        print("==== 交叉验证 FAIL：host_logger 认为这帧 CRC 不对（RTL 的 CRC 与 Python 不一致）====")
        return 1

    print("字段        TB 激励        Python 解出")
    print("-" * 44)
    bad = 0
    for k, want in EXPECT.items():
        got = f[k]
        flag = "" if abs(got - want) < 1e-6 else "   <-- 不一致"
        if flag:
            bad += 1
        print(f"{k:<11} {want:<14} {got}{flag}")

    print()
    if bad == 0:
        print("==== 交叉验证 PASS（%d 个字段全部一致，CRC 由 Python 独立复算通过）====" % len(EXPECT))
        return 0
    print("==== 交叉验证 FAIL：%d 个字段不一致 ====" % bad)
    return 1


if __name__ == "__main__":
    sys.exit(main())


