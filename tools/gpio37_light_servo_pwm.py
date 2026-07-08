#!/usr/bin/env python3
import argparse
import os
import time


GPIO = "37"
GPIO_DIR = f"/sys/class/gpio/gpio{GPIO}"
GPIO_VALUE = f"{GPIO_DIR}/value"
GPIO_DIRECTION = f"{GPIO_DIR}/direction"


def write(path, value):
    with open(path, "w", encoding="ascii") as f:
        f.write(str(value))


def setup_gpio():
    if not os.path.isdir(GPIO_DIR):
        try:
            write("/sys/class/gpio/export", GPIO)
            time.sleep(0.05)
        except OSError:
            pass
    write(GPIO_DIRECTION, "out")
    write(GPIO_VALUE, "0")


def pulse_train(pulse_us, duration_s, freq_hz):
    period_s = 1.0 / freq_hz
    high_s = pulse_us / 1_000_000.0
    low_s = max(0.0, period_s - high_s)
    end = time.monotonic() + duration_s
    while time.monotonic() < end:
        write(GPIO_VALUE, "1")
        time.sleep(high_s)
        write(GPIO_VALUE, "0")
        time.sleep(low_s)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pulse_us", type=int, nargs="?", default=1500)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--freq", type=float, default=50.0)
    args = parser.parse_args()

    if args.pulse_us == 0:
        setup_gpio()
        write(GPIO_VALUE, "0")
        print("GPIO37 low, light off")
        return

    if not 1100 <= args.pulse_us <= 1900:
        raise SystemExit("pulse_us must be 1100..1900, or 0 for off")

    setup_gpio()
    print(f"GPIO37 servo PWM: {args.pulse_us}us @ {args.freq}Hz for {args.duration}s")
    try:
        pulse_train(args.pulse_us, args.duration, args.freq)
    finally:
        write(GPIO_VALUE, "0")
        print("GPIO37 low, light off")


if __name__ == "__main__":
    main()
