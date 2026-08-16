#!/usr/bin/env python
"""Re-record safe limits for one joint and merge them into the limits file."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ladon_config import DEFAULT_FPS, JOINTS, JOINT_CHANNELS, SAFE_LIMITS_PATH, make_ladon, pose_from_observation


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", choices=JOINTS, required=True)
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS)
    parser.add_argument("--margin", type=float, default=2.0)
    return parser.parse_args()


def load_existing() -> dict:
    if SAFE_LIMITS_PATH.exists():
        return json.loads(SAFE_LIMITS_PATH.read_text())
    return {"robot_id": "ladon", "source": "scripts/record_one_safe_limit.py", "limits": {}}


def print_limit(joint: str, minimum: float, maximum: float) -> None:
    units = "pct" if JOINT_CHANNELS[joint] == "gripper" else "deg"
    print(f"{joint:16s} min={minimum:8.2f} max={maximum:8.2f} span={maximum - minimum:8.2f} {units}")


def main() -> None:
    args = parse_args()
    robot = make_ladon()
    robot.connect()
    try:
        print("Connected. Disabling torque so you can move the joint gently by hand.")
        robot.bus.disable_torque()
        input(f"Press Enter, then move physical {args.joint} through the range scripts may use...")

        pose = pose_from_observation(robot.get_observation())
        minimum = pose[args.joint]
        maximum = pose[args.joint]

        print("Sampling. Press Enter to stop.")
        while True:
            pose = pose_from_observation(robot.get_observation())
            minimum = min(minimum, pose[args.joint])
            maximum = max(maximum, pose[args.joint])
            print("\033[2J\033[H", end="")
            print_limit(args.joint, minimum, maximum)
            print("\nPress Enter to stop.")

            import select

            readable, _, _ = select.select([sys.stdin], [], [], 1.0 / max(args.fps, 0.1))
            if readable:
                sys.stdin.readline()
                break

        span = maximum - minimum
        margin = min(max(args.margin, 0.0), max(span / 3.0, 0.0))
        existing = load_existing()
        existing["joint_channel_map"] = JOINT_CHANNELS
        existing.setdefault("limits", {})[args.joint] = {
            "min": minimum + margin,
            "max": maximum - margin,
            "measured_min": minimum,
            "measured_max": maximum,
            "margin": margin,
        }
        SAFE_LIMITS_PATH.parent.mkdir(parents=True, exist_ok=True)
        SAFE_LIMITS_PATH.write_text(json.dumps(existing, indent=2) + "\n")

        print(f"\nSaved {args.joint} limits to {SAFE_LIMITS_PATH}")
        print_limit(args.joint, minimum + margin, maximum - margin)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
