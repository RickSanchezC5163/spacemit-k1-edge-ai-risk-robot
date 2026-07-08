#!/usr/bin/env python3
import argparse
import time

import serial


def move_packet(servo_id: int, pulse: int, duration_ms: int) -> bytes:
    servo_id = max(1, min(254, int(servo_id)))
    pulse = max(0, min(1000, int(pulse)))
    duration_ms = max(0, min(30000, int(duration_ms)))

    return bytes(
        [
            0x55,
            0x55,
            0x08,
            0x03,
            0x01,
            duration_ms & 0xFF,
            (duration_ms >> 8) & 0xFF,
            servo_id,
            pulse & 0xFF,
            (pulse >> 8) & 0xFF,
        ]
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Small smoke test for the bus servo controller."
    )
    parser.add_argument("--port", default="/dev/arm_bus")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--id", type=int, default=1)
    parser.add_argument("--center", type=int, default=500)
    parser.add_argument("--delta", type=int, default=30)
    parser.add_argument("--time-ms", type=int, default=800)
    parser.add_argument(
        "--run",
        action="store_true",
        help="Actually move the servo. Without this flag the command is a dry run.",
    )
    args = parser.parse_args()

    target = args.center + args.delta
    print(
        f"port={args.port} baud={args.baud} id={args.id} "
        f"center={args.center} target={target} time_ms={args.time_ms}"
    )

    if not args.run:
        print("Dry run only. Add --run after checking power, wiring, and clearance.")
        return 0

    with serial.Serial(args.port, args.baud, timeout=1) as ser:
        ser.write(move_packet(args.id, args.center, args.time_ms))
        time.sleep(max(args.time_ms / 1000.0, 0.5))
        ser.write(move_packet(args.id, target, args.time_ms))
        time.sleep(max(args.time_ms / 1000.0, 0.5))
        ser.write(move_packet(args.id, args.center, args.time_ms))
        time.sleep(max(args.time_ms / 1000.0, 0.5))

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
