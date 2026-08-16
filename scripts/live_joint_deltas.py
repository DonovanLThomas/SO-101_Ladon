#!/usr/bin/env python
"""Live joint-label monitor while you move the arm by hand."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ladon_config import JOINTS, JOINT_CHANNELS, make_ladon, pose_from_observation


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fps", type=float, default=5.0, help="Refresh rate for the live table.")
    parser.add_argument("--threshold", type=float, default=1.0, help="Delta size that marks a joint as moving.")
    return parser.parse_args()


def clear_screen() -> None:
    print("\033[2J\033[H", end="")


def print_table(baseline: dict[str, float], pose: dict[str, float], threshold: float) -> None:
    deltas = {joint: pose[joint] - baseline[joint] for joint in JOINTS}
    biggest = max(JOINTS, key=lambda joint: abs(deltas[joint]))

    print("Live joint deltas from baseline")
    print("Move one physical joint gently. Press Ctrl+C to stop.\n")
    print(f"{'label':16s} {'current':>10s} {'delta':>10s} {'units':>6s}  status")
    print("-" * 58)
    for joint in JOINTS:
        units = "pct" if JOINT_CHANNELS[joint] == "gripper" else "deg"
        delta = deltas[joint]
        marker = "<-- biggest" if joint == biggest and abs(delta) >= threshold else ""
        moving = "*" if abs(delta) >= threshold else " "
        print(f"{joint:16s} {pose[joint]:10.2f} {delta:10.2f} {units:>6s}  {moving} {marker}")


def main() -> None:
    args = parse_args()
    robot = make_ladon()
    robot.connect()
    try:
        print("Connected. Disabling torque so you can move joints gently by hand.")
        robot.bus.disable_torque()
        baseline = pose_from_observation(robot.get_observation())

        delay = 1.0 / max(args.fps, 0.1)
        while True:
            loop_start = time.perf_counter()
            pose = pose_from_observation(robot.get_observation())
            clear_screen()
            print_table(baseline, pose, args.threshold)
            elapsed = time.perf_counter() - loop_start
            time.sleep(max(delay - elapsed, 0.0))
    except KeyboardInterrupt:
        print("\nStopped live monitor.")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
