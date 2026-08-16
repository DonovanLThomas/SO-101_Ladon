# Ladon Joint Mapping Notes

Result from `python scripts/identify_joint_mapping.py`:

| Physical joint moved | LeRobot name that changed most |
| --- | --- |
| shoulder_pan | shoulder_pan |
| shoulder_lift | shoulder_lift |
| elbow_flex | elbow_flex |
| wrist_flex | wrist_flex |
| wrist_roll | gripper |
| gripper | wrist_roll |

This means physical `wrist_roll` and physical `gripper` are swapped relative to the standard SO-101 follower mapping.

The standard local LeRobot `SO101Follower` mapping is:

- `wrist_roll` -> motor ID 5
- `gripper` -> motor ID 6

Ladon's current behavior indicates those two physical motors are reversed. Do not run wrist-roll wave motions until IDs 5 and 6 are corrected and the arm is recalibrated.

Safe temporary motion tests:

```bash
python scripts/nudge_joint.py --joint wrist_flex --delta 2.0 --seconds 2 --return-home
python scripts/tiny_wave.py --joint wrist_flex --cycles 2 --amplitude 3.0
```

Likely repair path:

1. Use LeRobot's motor setup process to assign the physical wrist-roll motor to ID 5 and physical gripper motor to ID 6.
2. Recalibrate `ladon`.
3. Rerun `python scripts/identify_joint_mapping.py`.
4. Remove the `wrist_roll` entry from `BLOCKED_MOTION_JOINTS` in `ladon_config.py` only after the mapping is correct.
