#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
check_frame_decode.py —— 验证「RTL 真正吐出来的帧」能被「上位机解码器」正确还原

为什么需要这一步：
    host_logger.py 的自检是"Python 造帧 -> Python 解帧"，属于自证。
    如果 frame_pack 的字节序、CRC 覆盖范围、duty 的位拼接与上位机理解不一致，
    上板那天会表现为"一帧都收不到"，而且现场很难查。

    所以先用仿真把 RTL 的帧抓下来（sim/tb/tb_frame_bytes.sv），再用本脚本解码比对。
    这一步过了，才算 RTL <-> 上位机 真正联调过。

生成输入文件：
    xvlog -i rtl/pkg rtl/telemetry/frame_pack.v sim/tb/tb_frame_bytes.sv
    xelab tb_frame_bytes -s fb_sim -timescale 1ns/1ps
    xsim fb_sim -runall            # 在 work/ 下运行，产出 frame_bytes.bin（64 B = 2 帧）

然后：
    python sw/check_frame_decode.py frame_bytes.bin
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from host_logger import frames_from_stream   # noqa: E402

# tb_frame_bytes.sv 里 send_one() 驱动的两组值，逐字段对照
EXPECTED = [
    dict(seq=7, ts_ms=123456, tj_c=45.0390625, text_c=39.98828125, rh_pct=55.1,
         tset_c=50.0, duty=592, gear=3, state=0, f_mhz=100.0, p_mw=1200, age=2, flags=0),
    dict(seq=8, ts_ms=1, tj_c=-12.5, text_c=-3.25, rh_pct=40.0,
         tset_c=40.0, duty=7, gear=0, state=3, f_mhz=25.0, p_mw=600, age=0, flags=16),
]


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "frame_bytes.bin"
    if not os.path.exists(path):
        print("找不到 %s：先按文件头注释跑 tb_frame_bytes" % path, file=sys.stderr)
        return 2

    raw = open(path, "rb").read()
    frames = list(frames_from_stream(raw))
    print("bytes = %d, frames = %d (expect %d)" % (len(raw), len(frames), len(EXPECTED)))

    if len(frames) != len(EXPECTED):
        print("[FAIL] frame count mismatch", file=sys.stderr)
        return 1

    ok = True
    for i, (f, e) in enumerate(zip(frames, EXPECTED)):
        for k, v in e.items():
            got = f[k]
            good = abs(got - v) < 1e-6 if isinstance(v, float) else got == v
            if not good:
                ok = False
                print("  [FAIL] frame%d %s: got %r expect %r" % (i, k, got, v))
        print("  frame%d: seq=%-3d ts=%-7d tj=%8.4f text=%8.4f rh=%5.1f tset=%5.1f "
              "duty=%-4d gear=%d state=%d f=%5.1f p=%-5d age=%d flags=%d"
              % (i, f["seq"], f["ts_ms"], f["tj_c"], f["text_c"], f["rh_pct"],
                 f["tset_c"], f["duty"], f["gear"], f["state"], f["f_mhz"],
                 f["p_mw"], f["age"], f["flags"]))

    print("[%s] RTL frame <-> host_logger decoder" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
