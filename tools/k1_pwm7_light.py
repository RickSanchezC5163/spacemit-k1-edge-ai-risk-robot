#!/usr/bin/env python3
"""Control the GPIO37 lamp through the K1 PWM7 hardware controller."""

from __future__ import annotations

import argparse
import subprocess
from dataclasses import dataclass
from pathlib import Path


DEFAULT_HELPER = Path("/usr/local/sbin/k1-light-mode")
PERIOD_NS = 20_000_000
MIN_PULSE_US = 1100
MAX_PULSE_US = 1900


def brightness_to_pulse_us(brightness: float) -> int:
    value = max(0.0, min(100.0, float(brightness)))
    if value == 0:
        return 0
    return round(MIN_PULSE_US + (MAX_PULSE_US - MIN_PULSE_US) * value / 100.0)


@dataclass
class Pwm7Light:
    helper: Path = DEFAULT_HELPER

    def run(self, action: str) -> str:
        if action not in {"on", "off", "status"}:
            raise ValueError(f"unsupported light action: {action}")
        result = subprocess.run(
            ["sudo", "-n", str(self.helper), action],
            check=True,
            capture_output=True,
            text=True,
        )
        return result.stdout.strip()

    def off(self) -> None:
        self.run("off")

    def set_brightness(self, brightness: float) -> int:
        pulse_us = brightness_to_pulse_us(brightness)
        if pulse_us == 0:
            self.off()
            return 0
        if pulse_us != 1900:
            raise ValueError("installed K1 light helper supports only 100% brightness")
        self.run("on")
        return pulse_us

    def status(self) -> str:
        return self.run("status")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--helper", type=Path, default=DEFAULT_HELPER)
    subparsers = parser.add_subparsers(dest="command", required=True)
    set_parser = subparsers.add_parser("set")
    set_parser.add_argument("brightness", type=float)
    subparsers.add_parser("off")
    subparsers.add_parser("status")
    args = parser.parse_args()

    light = Pwm7Light(args.helper)
    if args.command == "set":
        pulse_us = light.set_brightness(args.brightness)
        print(f"PWM7 light brightness={args.brightness:g}% pulse={pulse_us}us period=20000us")
    elif args.command == "off":
        light.off()
        print("PWM7 light off")
    else:
        print(light.status())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
