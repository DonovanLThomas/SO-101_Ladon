#!/usr/bin/env python
"""Move one joint by a small relative amount from its measured position."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ladon_config import (
    ARM_JOINTS,
    DEFAULT_FPS,
    DEFAULT_NUDGE_SECONDS,
    MAX_NUDGE_DEG,
    clamp_abs,
    make_ladon,
    pose_from_observation,
    print_pose,
    require_motion_joint_allowed,
    slew_to_pose,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", choices=ARM_JOINTS, required=True)
    parser.add_argument("--delta", type=float, required=True, help="Relative change from the measured pose.")
    parser.add_argument("--seconds", type=float, default=DEFAULT_NUDGE_SECONDS)
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS)
    parser.add_argument("--return-home", action="store_true", help="Slew back to the starting pose before exit.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    require_motion_joint_allowed(args.joint)
    delta = clamp_abs(args.delta, MAX_NUDGE_DEG, "delta")

    robot = make_ladon()
    robot.connect()
    try:
        start_pose = pose_from_observation(robot.get_observation())
        target_pose = dict(start_pose)
        target_pose[args.joint] = start_pose[args.joint] + delta

        print("Starting pose:")
        print_pose(start_pose)
        print(f"\nNudging {args.joint} by {delta:.2f} deg.")
        slew_to_pose(robot, start_pose, target_pose, joints=[args.joint], seconds=args.seconds, fps=args.fps)

        if args.return_home:
            print(f"Returning {args.joint} to start.")
            slew_to_pose(robot, target_pose, start_pose, joints=[args.joint], seconds=args.seconds, fps=args.fps)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
