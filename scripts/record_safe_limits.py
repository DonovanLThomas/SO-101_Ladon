#!/usr/bin/env python
"""Record manual safe joint limits and save them for motion-script clamping."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ladon_config import (
    DEFAULT_FPS,
    JOINTS,
    JOINT_CHANNELS,
    SAFE_LIMITS_PATH,
    make_ladon,
    pose_from_observation,
    print_pose,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS)
    parser.add_argument("--margin", type=float, default=2.0, help="Stay this much inside measured min/max.")
    parser.add_argument(
        "--all-at-once",
        action="store_true",
        help="Record all joints during one continuous manual movement instead of one joint at a time.",
    )
    return parser.parse_args()


def empty_limits(start_pose: dict[str, float]) -> dict[str, dict[str, float]]:
    return {joint: {"min": start_pose[joint], "max": start_pose[joint]} for joint in JOINTS}


def update_limits(limits: dict[str, dict[str, float]], pose: dict[str, float]) -> None:
    for joint in JOINTS:
        limits[joint]["min"] = min(limits[joint]["min"], pose[joint])
        limits[joint]["max"] = max(limits[joint]["max"], pose[joint])


def print_limits(limits: dict[str, dict[str, float]]) -> None:
    print(f"{'label':16s} {'min':>10s} {'max':>10s} {'span':>10s} units")
    print("-" * 58)
    for joint in JOINTS:
        units = "pct" if JOINT_CHANNELS[joint] == "gripper" else "deg"
        minimum = limits[joint]["min"]
        maximum = limits[joint]["max"]
        print(f"{joint:16s} {minimum:10.2f} {maximum:10.2f} {maximum - minimum:10.2f} {units}")


def sample_until_enter(robot, limits: dict[str, dict[str, float]], fps: float) -> None:
    print("Sampling. Move gently. Press Enter to stop this recording step.")
    deadline = 0.0
    while True:
        pose = pose_from_observation(robot.get_observation())
        update_limits(limits, pose)
        now = time.perf_counter()
        if now >= deadline:
            print("\033[2J\033[H", end="")
            print_limits(limits)
            print("\nPress Enter to stop this step.")
            deadline = now + 0.25
        if sys.stdin in select_ready():
            sys.stdin.readline()
            return
        time.sleep(max(1.0 / fps, 0.01))


def select_ready():
    import select

    readable, _, _ = select.select([sys.stdin], [], [], 0)
    return readable


def apply_margin(limits: dict[str, dict[str, float]], margin: float) -> dict[str, dict[str, float]]:
    safe: dict[str, dict[str, float]] = {}
    for joint in JOINTS:
        minimum = limits[joint]["min"]
        maximum = limits[joint]["max"]
        span = maximum - minimum
        joint_margin = min(max(margin, 0.0), max(span / 3.0, 0.0))
        safe[joint] = {
            "min": minimum + joint_margin,
            "max": maximum - joint_margin,
            "measured_min": minimum,
            "measured_max": maximum,
            "margin": joint_margin,
        }
    return safe


def save_limits(limits: dict[str, dict[str, float]], margin: float) -> None:
    safe_limits = apply_margin(limits, margin)
    SAFE_LIMITS_PATH.parent.mkdir(parents=True, exist_ok=True)
    SAFE_LIMITS_PATH.write_text(
        json.dumps(
            {
                "robot_id": "ladon",
                "source": "scripts/record_safe_limits.py",
                "joint_channel_map": JOINT_CHANNELS,
                "limits": safe_limits,
            },
            indent=2,
        )
        + "\n"
    )


def main() -> None:
    args = parse_args()
    robot = make_ladon()
    robot.connect()
    try:
        print("Connected. Disabling torque so you can move joints gently by hand.")
        robot.bus.disable_torque()
        start_pose = pose_from_observation(robot.get_observation())
        limits = empty_limits(start_pose)

        print("\nStarting pose:")
        print_pose(start_pose)

        if args.all_at_once:
            input("\nPress Enter, then move all joints through the safe range you want to allow...")
            sample_until_enter(robot, limits, args.fps)
        else:
            print("\nEach step records all labels, but move only the named physical joint.")
            print("Do not force hard stops. Record the range you want scripts to use.")
            for joint in JOINTS:
                input(f"\nPress Enter to start recording physical {joint}...")
                sample_until_enter(robot, limits, args.fps)

        save_limits(limits, args.margin)
        print(f"\nSaved safe limits to {SAFE_LIMITS_PATH}")
        print("Future motion scripts will clamp targets to these limits.")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
