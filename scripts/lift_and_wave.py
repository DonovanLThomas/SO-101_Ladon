#!/usr/bin/env python
"""Lift from the current rested pose, wave gently, then return to the start pose."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ladon_config import (
    DEFAULT_BOTTOM_HOLD_SECONDS,
    DEFAULT_FPS,
    DEFAULT_LIFT_SECONDS,
    DEFAULT_RETURN_SECONDS,
    DEFAULT_WAVE_AMPLITUDE_DEG,
    DEFAULT_WAVE_CYCLES,
    MAX_ELBOW_LIFT_OFFSET_DEG,
    MAX_LIFT_OFFSET_DEG,
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


LIFT_JOINTS = ["shoulder_lift", "elbow_flex", "wrist_flex"]
RETURN_JOINTS = ["shoulder_lift", "elbow_flex", "wrist_flex", "shoulder_pan"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wave-joint", choices=["wrist_flex", "shoulder_pan"], default="wrist_flex")
    parser.add_argument("--cycles", type=int, default=DEFAULT_WAVE_CYCLES)
    parser.add_argument("--amplitude", type=float, default=DEFAULT_WAVE_AMPLITUDE_DEG)
    parser.add_argument("--shoulder-assist", type=float, default=0.0)
    parser.add_argument("--cycle-seconds", type=float, default=2.0)
    parser.add_argument("--lift-seconds", type=float, default=DEFAULT_LIFT_SECONDS)
    parser.add_argument(
        "--parallel-lift",
        action="store_true",
        help="Move shoulder, elbow, and wrist together instead of elbow first.",
    )
    parser.add_argument("--return-seconds", type=float, default=DEFAULT_RETURN_SECONDS)
    parser.add_argument("--bottom-hold-seconds", type=float, default=DEFAULT_BOTTOM_HOLD_SECONDS)
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS)
    parser.add_argument(
        "--shoulder-lift-offset",
        type=float,
        default=15.0,
        help="Relative shoulder_lift change from the measured bottom pose.",
    )
    parser.add_argument(
        "--elbow-flex-offset",
        type=float,
        default=-28.0,
        help="Relative elbow_flex change from the measured bottom pose.",
    )
    parser.add_argument(
        "--wrist-flex-offset",
        type=float,
        default=-8.0,
        help="Relative wrist_flex change from the measured bottom pose.",
    )
    return parser.parse_args()


def build_lift_pose(bottom_pose: dict[str, float], args: argparse.Namespace) -> dict[str, float]:
    offsets = {
        "shoulder_lift": clamp_abs(args.shoulder_lift_offset, MAX_LIFT_OFFSET_DEG, "shoulder-lift-offset"),
        "elbow_flex": clamp_abs(args.elbow_flex_offset, MAX_ELBOW_LIFT_OFFSET_DEG, "elbow-flex-offset"),
        "wrist_flex": clamp_abs(args.wrist_flex_offset, MAX_LIFT_OFFSET_DEG, "wrist-flex-offset"),
    }
    lifted_pose = dict(bottom_pose)
    for joint, offset in offsets.items():
        lifted_pose[joint] = bottom_pose[joint] + offset
    return lifted_pose


def hold_pose(robot, pose: dict[str, float], seconds: float, fps: float) -> None:
    end_time = time.perf_counter() + max(seconds, 0.0)
    while time.perf_counter() < end_time:
        loop_start = time.perf_counter()
        robot.send_action(action_from_pose(pose))
        sleep_step(loop_start, fps)


def wave_from_pose(robot, lifted_pose: dict[str, float], args: argparse.Namespace) -> None:
    require_motion_joint_allowed(args.wave_joint)
    amplitude = abs(clamp_abs(args.amplitude, MAX_WAVE_AMPLITUDE_DEG, "amplitude"))
    shoulder_assist = clamp_abs(args.shoulder_assist, MAX_SHOULDER_ASSIST_DEG, "shoulder-assist")
    cycles = max(1, args.cycles)
    total_seconds = max(0.5, args.cycle_seconds * cycles)
    steps = max(1, int(total_seconds * args.fps))

    print(f"\nWaving {args.wave_joint} +/- {amplitude:.2f} deg from lifted pose.")
    for step in range(steps + 1):
        loop_start = time.perf_counter()
        phase = 2.0 * math.pi * cycles * (step / steps)
        pose = {
            args.wave_joint: lifted_pose[args.wave_joint] + amplitude * math.sin(phase),
        }
        if shoulder_assist:
            pose["shoulder_pan"] = lifted_pose["shoulder_pan"] + shoulder_assist * math.sin(phase)
        robot.send_action(action_from_pose(pose, list(pose)))
        sleep_step(loop_start, args.fps)


def lift_to_pose(
    robot,
    bottom_pose: dict[str, float],
    lifted_pose: dict[str, float],
    args: argparse.Namespace,
) -> None:
    print(f"\nLifting over {args.lift_seconds:.1f}s.")
    if args.parallel_lift:
        slew_to_pose(
            robot,
            bottom_pose,
            lifted_pose,
            joints=LIFT_JOINTS,
            seconds=args.lift_seconds,
            fps=args.fps,
        )
        return

    elbow_pose = dict(bottom_pose)
    elbow_pose["elbow_flex"] = lifted_pose["elbow_flex"]

    elbow_seconds = max(args.lift_seconds * 0.60, 0.5)
    shoulder_seconds = max(args.lift_seconds - elbow_seconds, 0.5)

    print(f"Stage 1: elbow_flex to lift the forearm over {elbow_seconds:.1f}s.")
    slew_to_pose(
        robot,
        bottom_pose,
        elbow_pose,
        joints=["elbow_flex"],
        seconds=elbow_seconds,
        fps=args.fps,
    )

    print(f"Stage 2: shoulder_lift and wrist_flex over {shoulder_seconds:.1f}s.")
    slew_to_pose(
        robot,
        elbow_pose,
        lifted_pose,
        joints=["shoulder_lift", "wrist_flex"],
        seconds=shoulder_seconds,
        fps=args.fps,
    )


def return_to_bottom(robot, bottom_pose: dict[str, float], args: argparse.Namespace) -> None:
    current_pose = pose_from_observation(robot.get_observation())
    print(f"\nReturning to measured bottom pose over {args.return_seconds:.1f}s.")
    slew_to_pose(
        robot,
        current_pose,
        bottom_pose,
        joints=RETURN_JOINTS,
        seconds=args.return_seconds,
        fps=args.fps,
    )

    print(f"Holding bottom pose for {args.bottom_hold_seconds:.1f}s before disconnect.")
    hold_pose(robot, bottom_pose, args.bottom_hold_seconds, args.fps)


def main() -> None:
    args = parse_args()

    robot = make_ladon()
    robot.connect()
    try:
        bottom_pose = pose_from_observation(robot.get_observation())
        lifted_pose = build_lift_pose(bottom_pose, args)

        print("Measured bottom pose. The script will return here before disconnecting:")
        print_pose(bottom_pose)
        print("\nLift target:")
        print_pose(lifted_pose)

        try:
            lift_to_pose(robot, bottom_pose, lifted_pose, args)
            wave_from_pose(robot, lifted_pose, args)
            return_to_bottom(robot, bottom_pose, args)
        except KeyboardInterrupt:
            print("\nInterrupted. Attempting to return to the measured bottom pose.")
            return_to_bottom(robot, bottom_pose, args)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
