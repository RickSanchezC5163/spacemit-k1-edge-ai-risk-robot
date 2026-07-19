#!/usr/bin/env python3
"""Stationary smoke test for D435 raw capture and on-demand alignment."""

from __future__ import annotations

import argparse
import json
import resource
import time

from tools.k1_realsense_latest_frame import RealSenseLatestFrameSource


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-s", type=float, default=2.0)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--continuous-align", action="store_true")
    args = parser.parse_args()

    wall_started = time.perf_counter()
    cpu_started = time.process_time()
    source = RealSenseLatestFrameSource(fps=args.fps, slots=3).start()
    try:
        deadline = time.monotonic() + max(0.5, args.capture_s)
        continuous_alignments = 0
        last_frame_number = -1
        while time.monotonic() < deadline:
            if not args.continuous_align:
                time.sleep(max(0.0, deadline - time.monotonic()))
                break
            current = source.get_latest(copy=False)
            if current is not None and current.color_frame_number != last_frame_number:
                source.align_depth(current, copy=False)
                continuous_alignments += 1
                last_frame_number = current.color_frame_number
            else:
                time.sleep(0.002)
        before = source.status()
        frame = source.get_latest(copy=True)
        if frame is None:
            raise RuntimeError("D435 did not produce a color frame")
        after_rgb = source.status()
        aligned = source.align_depth(frame, copy=True)
        after_align = source.status()
        wall_s = time.perf_counter() - wall_started
        cpu_s = time.process_time() - cpu_started
        print(
            json.dumps(
                {
                    "before": before,
                    "after_rgb": after_rgb,
                    "after_align": after_align,
                    "color_shape": list(frame.color_bgr.shape),
                    "depth_shape": list(aligned.depth_raw.shape),
                    "same_frameset": frame.depth_frame_number == aligned.depth_frame_number,
                    "align_latency_ms": round(aligned.alignment_latency_ms, 3),
                    "continuous_alignments": continuous_alignments,
                    "process_cpu_percent": round(cpu_s / wall_s * 100.0, 2),
                    "max_rss_kib": int(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss),
                },
                sort_keys=True,
            )
        )
        return 0
    finally:
        source.stop()


if __name__ == "__main__":
    raise SystemExit(main())
