"""Shared settings and helpers for safely testing the SO-101 follower arm."""

from __future__ import annotations

import json
import time
from collections.abc import Mapping
from functools import lru_cache
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.utils.robot_utils import precise_sleep

ROBOT_PORT = "/dev/ttyACM0"
ROBOT_ID = "ladon"
REPO_ROOT = Path(__file__).resolve().parent
SAFE_LIMITS_PATH = REPO_ROOT / "config" / "safe_joint_limits.json"

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

# Temporary software remap for Ladon's current motor IDs.
# Physical wrist_roll is currently on the LeRobot gripper channel, and
# physical gripper is currently on the LeRobot wrist_roll channel.
JOINT_CHANNELS = {
    "shoulder_pan": "shoulder_pan",
    "shoulder_lift": "shoulder_lift",
    "elbow_flex": "elbow_flex",
    "wrist_flex": "wrist_flex",
    "wrist_roll": "gripper",
    "gripper": "wrist_roll",
}

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

BLOCKED_MOTION_JOINTS = {}


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
    return f"{JOINT_CHANNELS[joint]}{POSITION_KEY_SUFFIX}"


def pose_from_observation(observation: Mapping[str, float]) -> dict[str, float]:
    return {joint: float(observation[action_key(joint)]) for joint in JOINTS}


@lru_cache(maxsize=1)
def load_safe_limits() -> dict[str, dict[str, float]]:
    if not SAFE_LIMITS_PATH.exists():
        return {}
    data = json.loads(SAFE_LIMITS_PATH.read_text())
    return {
        joint: {"min": float(values["min"]), "max": float(values["max"])}
        for joint, values in data.get("limits", {}).items()
    }


def clamp_to_safe_limits(pose: Mapping[str, float], joints: list[str]) -> dict[str, float]:
    limits = load_safe_limits()
    clamped: dict[str, float] = {}
    for joint in joints:
        value = float(pose[joint])
        joint_limits = limits.get(joint)
        if not joint_limits:
            clamped[joint] = value
            continue

        safe_value = min(max(value, joint_limits["min"]), joint_limits["max"])
        if abs(safe_value - value) > 1e-6:
            print(
                f"{joint} target {value:.2f} is outside recorded safe limits "
                f"[{joint_limits['min']:.2f}, {joint_limits['max']:.2f}]; using {safe_value:.2f}."
            )
        clamped[joint] = safe_value
    return clamped


def action_from_pose(pose: Mapping[str, float], joints: list[str] | None = None) -> dict[str, float]:
    selected = JOINTS if joints is None else joints
    safe_pose = clamp_to_safe_limits(pose, selected)
    return {action_key(joint): safe_pose[joint] for joint in selected}


def print_pose(pose: Mapping[str, float]) -> None:
    for joint in JOINTS:
        units = "pct" if JOINT_CHANNELS[joint] == GRIPPER else "deg"
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
