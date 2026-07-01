#!/usr/bin/env python3
"""Decode Board1 0x301 per-motor position feedback from candump output.

Board1 local Motor ID mapping:
    motor 0 = arm 2-axis
    motor 1 = arm 3-axis
    motor 2 = arm 4-axis
    motor 3 = arm 5-axis

Usage:
    candump can0,301:7FF | python3 decode_board1_feedback.py

Payload:
    Byte0   = Local Motor ID, 0~3
    Byte1   = Flags
    Byte2~5 = current_pos_001deg, int32 little-endian
    Byte6   = error/fault code, 0 if none
    Byte7   = sequence counter or reserved
"""
from __future__ import annotations

import re
import sys
from typing import Dict, Optional, Tuple

FRAME_RE = re.compile(
    r"(?:\S+\s+)?(?P<canid>[0-9A-Fa-f]{3})\s+\[8\]\s+(?P<data>(?:[0-9A-Fa-f]{2}\s*){8})"
)


def i32_le(b0: int, b1: int, b2: int, b3: int) -> int:
    v = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
    if v & 0x80000000:
        v -= 0x100000000
    return v


def decode_flags(flags: int) -> str:
    names = []
    if flags & 0x01:
        names.append("valid")
    if flags & 0x02:
        names.append("homed_ready")
    if flags & 0x04:
        names.append("moving")
    if flags & 0x08:
        names.append("target_reached")
    reserved = (flags >> 4) & 0x0F
    if reserved:
        names.append(f"reserved=0x{reserved:X}")
    return ",".join(names) if names else "none"


def parse_line(line: str) -> Optional[Tuple[int, int, int, int]]:
    match = FRAME_RE.search(line)
    if not match:
        return None
    if int(match.group("canid"), 16) != 0x301:
        return None
    data = [int(x, 16) for x in match.group("data").split()]
    if len(data) != 8:
        return None

    motor_id = data[0]
    flags = data[1]
    pos_raw = i32_le(data[2], data[3], data[4], data[5])
    error = data[6]
    seq = data[7]
    return motor_id, flags, pos_raw, error, seq


def main() -> int:
    latest: Dict[int, Tuple[float, int, int, int]] = {}
    axis_name = {0: "arm2", 1: "arm3", 2: "arm4", 3: "arm5"}

    for line in sys.stdin:
        parsed = parse_line(line)
        if parsed is None:
            continue

        motor_id, flags, pos_raw, error, seq = parsed
        if motor_id > 3:
            print(f"ignore unknown Board1 motor_id={motor_id} line={line.strip()}")
            continue

        latest[motor_id] = (pos_raw / 100.0, flags, error, seq)

        print(
            f"Board1 0x301 motor={motor_id}({axis_name[motor_id]}) "
            f"pos={pos_raw / 100.0:.2f} deg "
            f"flags=0x{flags:02X} ({decode_flags(flags)}) "
            f"error={error} seq={seq}"
        )

        if all(i in latest for i in range(4)):
            positions = ", ".join(f"m{i}({axis_name[i]})={latest[i][0]:.2f}" for i in range(4))
            seqs = ", ".join(f"m{i}:{latest[i][3]}" for i in range(4))
            print(f"Board1 full set: {positions} deg | seq {seqs}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
