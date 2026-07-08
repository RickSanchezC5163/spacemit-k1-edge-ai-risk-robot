#!/usr/bin/env python3
import argparse
import math
import sys


class StaticGuardModel:
    def __init__(self, args):
        self.hard_stop_m = args.hard_stop_m
        self.warning_m = args.warning_m
        self.emergency_stop_m = args.emergency_stop_m
        self.approach_stop_m = args.approach_stop_m
        self.approach_rate_stop_mps = args.approach_rate_stop_mps
        self.ttc_stop_s = args.ttc_stop_s
        self.hard_stop_latch_s = args.hard_stop_latch_s
        self.min_front_valid_count = args.min_front_valid_count
        self.state = "clear"
        self.latch_until = 0.0

    def update(self, now, front_min, front_p10, valid_count, approach_rate):
        ttc = math.inf
        if front_p10 is not None and approach_rate > 1e-6:
            ttc = front_p10 / approach_rate

        hard = False
        if front_min is not None and front_min <= self.emergency_stop_m:
            hard = True
        if front_p10 is not None and valid_count >= self.min_front_valid_count:
            hard = hard or front_p10 <= self.hard_stop_m
            hard = hard or (front_p10 < self.approach_stop_m and approach_rate > self.approach_rate_stop_mps)
            hard = hard or ttc < self.ttc_stop_s

        if hard:
            self.state = "hard_stop"
            self.latch_until = max(self.latch_until, now + self.hard_stop_latch_s)
        elif self.state == "hard_stop" and now < self.latch_until:
            pass
        elif front_p10 is not None and front_p10 < self.warning_m:
            self.state = "warning"
        else:
            self.state = "clear"

        return {
            "state": self.state,
            "front_min": front_min,
            "front_p10": front_p10,
            "valid_count": valid_count,
            "approach_rate_mps": approach_rate,
            "ttc_s": None if math.isinf(ttc) else ttc,
            "latch_remaining_s": max(0.0, self.latch_until - now),
        }


def assert_state(label, actual, expected):
    if actual["state"] != expected:
        raise AssertionError(f"{label}: expected {expected}, got {actual}")
    print(f"PASS {label}: {actual}")


def main():
    parser = argparse.ArgumentParser(
        description="Static checks for scan_safety_guard_node decision thresholds. Sends no ROS commands."
    )
    parser.add_argument("--hard-stop-m", type=float, default=1.00)
    parser.add_argument("--warning-m", type=float, default=1.60)
    parser.add_argument("--emergency-stop-m", type=float, default=0.45)
    parser.add_argument("--approach-stop-m", type=float, default=1.60)
    parser.add_argument("--approach-rate-stop-mps", type=float, default=0.35)
    parser.add_argument("--ttc-stop-s", type=float, default=1.20)
    parser.add_argument("--hard-stop-latch-s", type=float, default=1.50)
    parser.add_argument("--min-front-valid-count", type=int, default=3)
    args = parser.parse_args()

    guard = StaticGuardModel(args)
    assert_state(
        "2.0m clear",
        guard.update(0.0, front_min=1.95, front_p10=2.0, valid_count=20, approach_rate=0.0),
        "clear",
    )
    assert_state(
        "1.5m warning",
        guard.update(0.1, front_min=1.45, front_p10=1.5, valid_count=20, approach_rate=0.0),
        "warning",
    )
    assert_state(
        "1.0m hard_stop",
        guard.update(0.2, front_min=0.95, front_p10=1.0, valid_count=20, approach_rate=0.0),
        "hard_stop",
    )
    assert_state(
        "hard_stop latch",
        guard.update(0.7, front_min=1.95, front_p10=2.0, valid_count=20, approach_rate=0.0),
        "hard_stop",
    )
    assert_state(
        "latch released to clear",
        guard.update(1.8, front_min=1.95, front_p10=2.0, valid_count=20, approach_rate=0.0),
        "clear",
    )
    assert_state(
        "ttc hard_stop",
        guard.update(2.0, front_min=1.2, front_p10=1.2, valid_count=20, approach_rate=1.1),
        "hard_stop",
    )
    print("OK scan safety guard static checks passed; no ROS motion was sent.")


if __name__ == "__main__":
    try:
        main()
    except AssertionError as exc:
        print(f"FAIL {exc}", file=sys.stderr)
        sys.exit(1)
