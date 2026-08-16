#!/usr/bin/env python
"""Disable torque and identify which named joint changes when you move each physical joint."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ladon_config import JOINTS, make_ladon, pose_from_observation, print_pose


PHYSICAL_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--top", type=int, default=3, help="How many changing names to show per movement.")
    return parser.parse_args()


def print_biggest_changes(before: dict[str, float], after: dict[str, float], top: int) -> None:
    deltas = sorted(
        ((joint, after[joint] - before[joint], before[joint], after[joint]) for joint in JOINTS),
        key=lambda item: abs(item[1]),
        reverse=True,
    )
    for joint, delta, start, end in deltas[:top]:
        units = "pct" if joint == "gripper" else "deg"
        print(f"  {joint:16s} delta={delta:8.2f} {units}   {start:8.2f} -> {end:8.2f}")


def main() -> None:
    args = parse_args()
    robot = make_ladon()
    robot.connect()
    try:
        print("Connected. Disabling torque so you can move joints gently by hand.")
        robot.bus.disable_torque()

        baseline = pose_from_observation(robot.get_observation())
        print("\nBaseline pose:")
        print_pose(baseline)

        print("\nMove only the named physical joint a small amount, then press Enter.")
        print("Keep movements gentle and away from mechanical limits.")

        current = baseline
        for physical_joint in PHYSICAL_JOINTS:
            input(f"\nMove physical {physical_joint}, then press Enter...")
            new_pose = pose_from_observation(robot.get_observation())
            print(f"Observed changes after moving physical {physical_joint}:")
            print_biggest_changes(current, new_pose, args.top)
            current = new_pose

        print("\nDone. Use the largest changing LeRobot name for each physical joint.")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
