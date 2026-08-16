# SO-101 Ladon

Small, editable scripts for safely testing the SO-101 follower arm named `ladon`.

This repo assumes:

- Jetson Orin Nano
- LeRobot checkout at `~/lerobot`
- Conda env named `lerobot`
- Follower arm only, no leader arm
- Robot port `/dev/ttyACM0`
- Robot id `ladon`
- Existing calibration in the LeRobot Hugging Face cache

## Safety Checklist

Before running motion scripts:

1. Keep one hand near robot power.
2. Start with the arm in a rested, non-stressed pose.
3. Make sure no cables, fingers, tools, or table edges are in the motion path.
4. Run `read_pose.py` before motion.
5. Use `live_joint_deltas.py` if a joint label or direction seems unclear.

These scripts always read the current pose first and command small relative movements from that measured pose.

## Setup

From this repo:

```bash
conda activate lerobot
cd ~/lerobot/SO-101_Ladon
```

Quick import check:

```bash
python -c "from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig; print('ok')"
```

## Commands

Read all joints:

```bash
python scripts/read_pose.py
```

Record your own safe movement limits:

```bash
python scripts/record_safe_limits.py
```

This disables torque and records the min/max LeRobot labels while you gently move each physical joint through the range you want scripts to be allowed to use. It saves `config/safe_joint_limits.json`, and future motion scripts clamp targets to those limits.

Run the full motion: lift from the measured rested pose, wave gently, then return to that same rested pose before disconnecting:

```bash
python scripts/lift_and_wave.py --shoulder-lift-offset 10 --elbow-flex-offset -45 --wrist-flex-offset -5 --amplitude 2 --lift-seconds 6
```

By default the lift is staged: `elbow_flex` moves first, then `shoulder_lift` and `wrist_flex` follow. This tends to get the forearm up before the shoulder carries it higher.

If the lift direction looks wrong, stop with `Ctrl+C`. The script will attempt to return to the measured bottom pose, but keep one hand near power.

For a taller lift, increase elbow first while keeping shoulder and wrist smaller:

```bash
python scripts/lift_and_wave.py --shoulder-lift-offset 10 --elbow-flex-offset -60 --wrist-flex-offset -5 --amplitude 2 --lift-seconds 7
```

If one recorded limit is too tight, re-record only that joint:

```bash
python scripts/record_one_safe_limit.py --joint elbow_flex
```

## If A Joint Name Looks Wrong

For a live view while you move the arm by hand:

```bash
python scripts/live_joint_deltas.py
```

Current Ladon finding: physical `wrist_roll` and physical `gripper` are swapped in the LeRobot channels. Temporary repo behavior: `ladon_config.py` remaps physical `wrist_roll` to the LeRobot `gripper` channel, and physical `gripper` to the LeRobot `wrist_roll` channel. This lets scripts use the physical labels for now. Fix the motor IDs and recalibrate later, then remove the remap. See `docs/joint_mapping_ladon.md`.

## Notes

- Arm joints are in degrees because `use_degrees=True`.
- Because of the temporary wrist/gripper remap, printed units follow the underlying LeRobot channel until the motor IDs are fixed.
- `max_relative_target=2.0` is enabled in the shared config so LeRobot clips large per-command jumps.
- If the port or robot id changes, edit `ladon_config.py`.
