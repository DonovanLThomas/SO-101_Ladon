#!/usr/bin/env python
"""Run a very small wrist-led wave from the current measured pose."""

from __future__ import annotations

import argparse
import math
import time
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ladon_config import (
    ARM_JOINTS,
    DEFAULT_FPS,
    DEFAULT_WAVE_AMPLITUDE_DEG,
    DEFAULT_WAVE_CYCLES,
    MAX_SHOULDER_ASSIST_DEG,
    MAX_WAVE_AMPLITUDE_DEG,
    action_from_pose,
    clamp_abs,
    make_ladon,
    pose_from_observation,
    print_pose,
    require_motion_joint_allowed,
    sleep_step,
    slew_to_pose,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", choices=ARM_JOINTS, required=True, help="Named joint to wave.")
    parser.add_argument("--cycles", type=int, default=DEFAULT_WAVE_CYCLES)
    parser.add_argument("--amplitude", type=float, default=DEFAULT_WAVE_AMPLITUDE_DEG)
    parser.add_argument(
        "--shoulder-assist",
        type=float,
        default=0.0,
        help="Optional small shoulder_pan assist in degrees. Default keeps shoulder still.",
    )
    parser.add_argument("--cycle-seconds", type=float, default=2.0)
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS)
    parser.add_argument("--return-home-seconds", type=float, default=2.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    require_motion_joint_allowed(args.joint)
    amplitude = abs(clamp_abs(args.amplitude, MAX_WAVE_AMPLITUDE_DEG, "amplitude"))
    shoulder_assist = clamp_abs(args.shoulder_assist, MAX_SHOULDER_ASSIST_DEG, "shoulder-assist")
    cycles = max(1, args.cycles)

    robot = make_ladon()
    robot.connect()
    try:
        base_pose = pose_from_observation(robot.get_observation())
        print("Wave baseline pose:")
        print_pose(base_pose)
        print(f"\nWaving {args.joint} +/- {amplitude:.2f} deg for {cycles} cycle(s).")

        total_seconds = max(0.5, args.cycle_seconds * cycles)
        steps = max(1, int(total_seconds * args.fps))

        for step in range(steps + 1):
            loop_start = time.perf_counter()
            phase = 2.0 * math.pi * cycles * (step / steps)
            pose = {
                args.joint: base_pose[args.joint] + amplitude * math.sin(phase),
            }
            if shoulder_assist:
                pose["shoulder_pan"] = base_pose["shoulder_pan"] + shoulder_assist * math.sin(phase)
            robot.send_action(action_from_pose(pose, list(pose)))
            sleep_step(loop_start, args.fps)

        current_pose = pose_from_observation(robot.get_observation())
        return_joints = [args.joint]
        if shoulder_assist:
            return_joints.append("shoulder_pan")
        slew_to_pose(
            robot,
            current_pose,
            base_pose,
            joints=return_joints,
            seconds=args.return_home_seconds,
            fps=args.fps,
        )
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
