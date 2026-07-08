#!/usr/bin/env python3
import argparse
import os
import sys
import termios
import time


FRAME = bytes([0x7B, 0, 0, 0, 0, 0, 0, 0, 0, 0x7B, 0x7D])
BAUD_RATES = {
    9600: termios.B9600,
    57600: termios.B57600,
    115200: termios.B115200,
    460800: termios.B460800,
}


def configure_serial(fd: int, baud: int) -> None:
    if baud not in BAUD_RATES:
        raise ValueError(f"unsupported baud: {baud}")
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = BAUD_RATES[baud] | termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = BAUD_RATES[baud]
    attrs[5] = BAUD_RATES[baud]
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def main():
    parser = argparse.ArgumentParser(
        description="Send repeated C30D ROS-protocol zero-speed frames directly to serial."
    )
    parser.add_argument("--port", default="/dev/base_controller")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=100.0)
    args = parser.parse_args()

    print("SAFETY ZERO ONLY")
    print("- This writes only the C30D zero-speed frame.")
    print("- Stop ROS base driver first so the serial port is not busy.")
    print(f"- Port: {args.port}, baud: {args.baud}, duration: {args.duration:.1f}s")

    fd = None
    try:
        fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        configure_serial(fd, args.baud)
        period = 1.0 / max(1.0, args.rate)
        end_time = time.monotonic() + max(0.1, args.duration)
        count = 0
        while time.monotonic() < end_time:
            os.write(fd, FRAME)
            count += 1
            time.sleep(period)
        os.write(fd, FRAME)
        print(f"Sent {count + 1} zero frames.")
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        sys.exit(1)
    finally:
        if fd is not None:
            os.close(fd)


if __name__ == "__main__":
    main()
