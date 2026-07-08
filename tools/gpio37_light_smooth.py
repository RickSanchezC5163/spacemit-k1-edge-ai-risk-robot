#!/usr/bin/env python3
import argparse
import math
import os
import signal
import sys
import time


GPIO = "37"
GPIO_DIR = f"/sys/class/gpio/gpio{GPIO}"
GPIO_VALUE = f"{GPIO_DIR}/value"
GPIO_DIRECTION = f"{GPIO_DIR}/direction"

MIN_US = 1100
MAX_US = 1900
DEFAULT_FREQ = 50.0

running = True


def stop_handler(signum, frame):
    global running
    running = False


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


def set_low():
    setup_gpio()
    write(GPIO_VALUE, "0")


def brightness_to_pulse(brightness):
    b = max(0.0, min(100.0, float(brightness)))
    return MIN_US + (MAX_US - MIN_US) * (b / 100.0)


def ease_in_out(t):
    t = max(0.0, min(1.0, t))
    return 0.5 - 0.5 * math.cos(math.pi * t)


class SoftServoPwm:
    def __init__(self, freq_hz):
        self.period_s = 1.0 / freq_hz

    def one_frame(self, pulse_us):
        high_s = max(0.0, min(self.period_s, pulse_us / 1_000_000.0))
        low_s = max(0.0, self.period_s - high_s)
        write(GPIO_VALUE, "1")
        time.sleep(high_s)
        write(GPIO_VALUE, "0")
        time.sleep(low_s)

    def hold(self, pulse_us, duration_s):
        end = None if duration_s < 0 else time.monotonic() + duration_s
        while running and (end is None or time.monotonic() < end):
            self.one_frame(pulse_us)

    def ramp(self, start_us, end_us, duration_s):
        if duration_s <= 0:
            self.one_frame(end_us)
            return
        start = time.monotonic()
        while running:
            elapsed = time.monotonic() - start
            t = elapsed / duration_s
            if t >= 1.0:
                break
            k = ease_in_out(t)
            pulse_us = start_us + (end_us - start_us) * k
            self.one_frame(pulse_us)
        if running:
            self.one_frame(end_us)


def cmd_set(args):
    setup_gpio()
    pwm = SoftServoPwm(args.freq)
    start_us = brightness_to_pulse(args.start) if args.start is not None else brightness_to_pulse(0)
    target_us = brightness_to_pulse(args.brightness)
    print(
        f"GPIO37 light ramp: {args.start if args.start is not None else 0:.1f}% -> "
        f"{args.brightness:.1f}% in {args.ramp:.2f}s, hold={args.hold}s",
        flush=True,
    )
    pwm.ramp(start_us, target_us, args.ramp)
    pwm.hold(target_us, args.hold)
    set_low()
    print("GPIO37 low, light off", flush=True)


def cmd_breathe(args):
    setup_gpio()
    pwm = SoftServoPwm(args.freq)
    lo = brightness_to_pulse(args.low)
    hi = brightness_to_pulse(args.high)
    print(
        f"GPIO37 light breathe: {args.low:.1f}% <-> {args.high:.1f}%, "
        f"ramp={args.ramp:.2f}s, cycles={args.cycles}",
        flush=True,
    )
    count = 0
    while running and (args.cycles < 0 or count < args.cycles):
        pwm.ramp(lo, hi, args.ramp)
        pwm.ramp(hi, lo, args.ramp)
        count += 1
    set_low()
    print("GPIO37 low, light off", flush=True)


def cmd_off(args):
    set_low()
    print("GPIO37 low, light off", flush=True)


def main():
    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    parser = argparse.ArgumentParser(description="Smooth GPIO37 light control for 1100-1900us PWM lights.")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_set = sub.add_parser("set", help="ramp to a target brightness and hold")
    p_set.add_argument("brightness", type=float, help="target brightness, 0..100")
    p_set.add_argument("--start", type=float, default=None, help="start brightness, default 0")
    p_set.add_argument("--ramp", type=float, default=2.0, help="ramp duration in seconds")
    p_set.add_argument("--hold", type=float, default=-1.0, help="hold seconds; -1 means forever")
    p_set.add_argument("--freq", type=float, default=DEFAULT_FREQ)
    p_set.set_defaults(func=cmd_set)

    p_breathe = sub.add_parser("breathe", help="smooth breathing brightness pattern")
    p_breathe.add_argument("--low", type=float, default=0.0)
    p_breathe.add_argument("--high", type=float, default=50.0)
    p_breathe.add_argument("--ramp", type=float, default=2.5)
    p_breathe.add_argument("--cycles", type=int, default=-1, help="-1 means forever")
    p_breathe.add_argument("--freq", type=float, default=DEFAULT_FREQ)
    p_breathe.set_defaults(func=cmd_breathe)

    p_off = sub.add_parser("off", help="force GPIO37 low")
    p_off.set_defaults(func=cmd_off)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
