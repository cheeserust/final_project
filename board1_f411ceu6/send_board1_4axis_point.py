#!/usr/bin/env python3
"""
Send one Board1 4-axis trajectory point according to the integrated 0x101 protocol.

Board1 local Motor ID mapping:
    motor 0 = arm 2-axis
    motor 1 = arm 3-axis
    motor 2 = arm 4-axis
    motor 3 = arm 5-axis

Example:
    python3 send_board1_4axis_point.py can0 3000 -8000 -9000 -17000 --speed 1000 --duration 10

Target positions are 0.01 degree units by default:
    3000 = 30.00 deg
Duration is in 5 ms units:
    10 = 50 ms
"""
import argparse
import socket
import struct
import time

CAN_ID_BOARD1_MOVE = 0x101
CTRL_EXECUTE = 0x80
CTRL_RELATIVE = 0x40
CTRL_STEP_MODE = 0x20


def make_board1_move_frame(motor_id: int, target_pos: int, speed: int, duration_5ms: int,
                           relative: bool = False, step_mode: bool = False):
    if not 0 <= motor_id <= 3:
        raise ValueError("motor_id must be 0..3")
    if not -2_147_483_648 <= target_pos <= 2_147_483_647:
        raise ValueError("target_pos must fit int32")
    if not 0 <= speed <= 0xFFFF:
        raise ValueError("speed must fit uint16")
    if not 0 <= duration_5ms <= 0xFF:
        raise ValueError("duration_5ms must fit uint8")

    b0 = CTRL_EXECUTE | motor_id
    if relative:
        b0 |= CTRL_RELATIVE
    if step_mode:
        b0 |= CTRL_STEP_MODE

    data = struct.pack("<BiHB", b0, target_pos, speed, duration_5ms)
    return CAN_ID_BOARD1_MOVE, data


def send_can(sock: socket.socket, can_id: int, data: bytes):
    frame = struct.pack("=IB3x8s", can_id, len(data), data.ljust(8, b"\x00"))
    sock.send(frame)


def data_to_hex(data: bytes) -> str:
    return "".join(f"{b:02X}" for b in data)


def main():
    parser = argparse.ArgumentParser(description="Send Board1 4-frame 0x101 trajectory point for arm axes 2~5")
    parser.add_argument("ifname", help="SocketCAN interface, e.g. can0")
    parser.add_argument("targets", nargs=4, type=int,
                        help="target positions for Board1 motors 0..3 = arm axes 2~5. Unit: 0.01 degree")
    parser.add_argument("--speed", type=int, default=1000, help="speed in 0.01 deg/s, default 1000")
    parser.add_argument("--duration", type=int, default=10, help="duration in 5 ms units, default 10")
    parser.add_argument("--relative", action="store_true", help="set relative flag on all 4 frames")
    parser.add_argument("--step-mode", action="store_true", help="interpret target positions as steps")
    parser.add_argument("--gap", type=float, default=0.001,
                        help="gap between frames in seconds. Keep total under 20 ms. Default 0.001")
    args = parser.parse_args()

    if args.gap * 3.0 > 0.020:
        raise SystemExit("frame gap is too long: the 4 frames must fit within 20 ms")

    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((args.ifname,))

    for motor_id in range(4):
        can_id, data = make_board1_move_frame(
            motor_id=motor_id,
            target_pos=args.targets[motor_id],
            speed=args.speed,
            duration_5ms=args.duration,
            relative=args.relative,
            step_mode=args.step_mode,
        )
        print(f"cansend {args.ifname} {can_id:03X}#{data_to_hex(data)}")
        send_can(sock, can_id, data)
        time.sleep(args.gap)


if __name__ == "__main__":
    main()
