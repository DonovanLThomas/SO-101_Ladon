"""Shared settings and helpers for safely testing the SO-101 follower arm."""

from __future__ import annotations

import time
from collections.abc import Mapping

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.utils.robot_utils import precise_sleep

ROBOT_PORT = "/dev/ttyACM0"
ROBOT_ID = "ladon"

ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
GRIPPER = "gripper"
JOINTS = [*ARM_JOINTS, GRIPPER]

POSITION_KEY_SUFFIX = ".pos"

# LeRobot's SO follower uses degrees for arm joints when use_degrees=True.
# The gripper is normalized separately to 0..100.
MAX_RELATIVE_TARGET = 2.0
DEFAULT_FPS = 15.0
DEFAULT_NUDGE_SECONDS = 2.0
DEFAULT_HOLD_SECONDS = 3.0
DEFAULT_WAVE_AMPLITUDE_DEG = 3.0
DEFAULT_WAVE_CYCLES = 2
DEFAULT_LIFT_SECONDS = 4.0
DEFAULT_RETURN_SECONDS = 4.0
DEFAULT_BOTTOM_HOLD_SECONDS = 1.5
MAX_NUDGE_DEG = 5.0
MAX_WAVE_AMPLITUDE_DEG = 8.0
MAX_SHOULDER_ASSIST_DEG = 2.0
MAX_LIFT_OFFSET_DEG = 25.0
MAX_ELBOW_LIFT_OFFSET_DEG = 45.0

# Current hardware diagnostic result:
# physical wrist_roll reports through the LeRobot "gripper" channel, and
# physical gripper reports through the LeRobot "wrist_roll" channel.
# Block wrist_roll commands until motor IDs 5 and 6 are corrected and the arm is recalibrated.
BLOCKED_MOTION_JOINTS = {
    "wrist_roll": "Detected mapping swap: wrist_roll commands move the physical gripper on this arm.",
}


def make_ladon() -> SO101Follower:
    """Create the follower with conservative relative-target clipping."""
    config = SO101FollowerConfig(
        port=ROBOT_PORT,
        id=ROBOT_ID,
        use_degrees=True,
        max_relative_target=MAX_RELATIVE_TARGET,
    )
    return SO101Follower(config)


def action_key(joint: str) -> str:
    return f"{joint}{POSITION_KEY_SUFFIX}"


def pose_from_observation(observation: Mapping[str, float]) -> dict[str, float]:
    return {joint: float(observation[action_key(joint)]) for joint in JOINTS}


def action_from_pose(pose: Mapping[str, float], joints: list[str] | None = None) -> dict[str, float]:
    selected = JOINTS if joints is None else joints
    return {action_key(joint): float(pose[joint]) for joint in selected}


def print_pose(pose: Mapping[str, float]) -> None:
    for joint in JOINTS:
        units = "pct" if joint == GRIPPER else "deg"
        print(f"{joint:16s} {pose[joint]:8.2f} {units}")


def clamp_abs(value: float, limit: float, label: str) -> float:
    if abs(value) <= limit:
        return value

    clamped = limit if value > 0 else -limit
    print(f"{label} {value:.2f} exceeds safe limit {limit:.2f}; using {clamped:.2f}.")
    return clamped


def require_motion_joint_allowed(joint: str) -> None:
    reason = BLOCKED_MOTION_JOINTS.get(joint)
    if reason:
        raise SystemExit(f"Refusing to command {joint}. {reason}")


def sleep_step(start_time: float, fps: float) -> None:
    precise_sleep(max(1.0 / fps - (time.perf_counter() - start_time), 0.0))


def slew_to_pose(
    robot: SO101Follower,
    start_pose: Mapping[str, float],
    target_pose: Mapping[str, float],
    *,
    joints: list[str],
    seconds: float,
    fps: float = DEFAULT_FPS,
) -> None:
    """Linearly move selected joints from start_pose to target_pose."""
    steps = max(1, int(seconds * fps))
    for step in range(1, steps + 1):
        loop_start = time.perf_counter()
        alpha = step / steps
        pose = {
            joint: float(start_pose[joint]) + alpha * (float(target_pose[joint]) - float(start_pose[joint]))
            for joint in joints
        }
        robot.send_action(action_from_pose(pose, joints))
        sleep_step(loop_start, fps)
