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
5. Run one tiny nudge before trying `tiny_wave.py`.

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

Hold the current measured pose for 3 seconds:

```bash
python scripts/hold_pose.py --seconds 3
```

Record your own safe movement limits:

```bash
python scripts/record_safe_limits.py
```

This disables torque and records the min/max LeRobot labels while you gently move each physical joint through the range you want scripts to be allowed to use. It saves `config/safe_joint_limits.json`, and future motion scripts clamp targets to those limits.

Nudge one joint by a tiny relative amount and return:

```bash
python scripts/nudge_joint.py --joint shoulder_pan --delta 1.0 --seconds 2 --return-home
python scripts/nudge_joint.py --joint wrist_flex --delta 2.0 --seconds 2 --return-home
```

Run the smallest currently safe wrist-flex wave:

```bash
python scripts/tiny_wave.py --joint wrist_flex --cycles 2 --amplitude 3.0
```

Optional, after the wrist-only wave is smooth:

```bash
python scripts/tiny_wave.py --joint wrist_flex --cycles 2 --amplitude 3.0 --shoulder-assist 1.0
```

Lift from the measured rested pose, wave gently, then return to that same rested pose before disconnecting:

```bash
python scripts/lift_and_wave.py
```

By default the lift is staged: `elbow_flex` moves first, then `shoulder_lift` and `wrist_flex` follow. This tends to get the forearm up before the shoulder carries it higher.

If the lift direction looks wrong, stop with `Ctrl+C`. The script still returns through `finally`, but keep one hand near power. Try smaller or opposite offsets:

```bash
python scripts/lift_and_wave.py --shoulder-lift-offset 8 --elbow-flex-offset -8 --wrist-flex-offset -4
python scripts/lift_and_wave.py --shoulder-lift-offset -8 --elbow-flex-offset 8 --wrist-flex-offset 4
```

For a taller lift, increase elbow first while keeping shoulder and wrist smaller:

```bash
python scripts/lift_and_wave.py --shoulder-lift-offset 10 --elbow-flex-offset -28 --wrist-flex-offset -5 --amplitude 2
```

## If A Joint Name Looks Wrong

If commanding one named joint moves a different physical joint, stop motion tests and run:

```bash
python scripts/identify_joint_mapping.py
```

The script disables torque, asks you to gently move one physical joint at a time, and prints which LeRobot joint name changed most. Use that output before changing wave settings.

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
