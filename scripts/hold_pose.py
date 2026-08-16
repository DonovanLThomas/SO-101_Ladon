#!/usr/bin/env python
"""Latch the current pose and resend it briefly."""

from __future__ import annotations

import argparse
import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ladon_config import (
    DEFAULT_FPS,
    DEFAULT_HOLD_SECONDS,
    action_from_pose,
    make_ladon,
    pose_from_observation,
    print_pose,
    sleep_step,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=DEFAULT_HOLD_SECONDS)
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    robot = make_ladon()
    robot.connect()
    try:
        pose = pose_from_observation(robot.get_observation())
        print("Holding measured pose:")
        print_pose(pose)

        end_time = time.perf_counter() + max(args.seconds, 0.0)
        while time.perf_counter() < end_time:
            loop_start = time.perf_counter()
            robot.send_action(action_from_pose(pose))
            sleep_step(loop_start, args.fps)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
