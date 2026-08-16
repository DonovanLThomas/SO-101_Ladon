# Ladon Joint Mapping Notes

Observed manual mapping:

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

Ladon's current behavior indicates those two physical motors are reversed. For now, this repo uses a temporary software remap in `ladon_config.py`:

- physical `wrist_roll` -> LeRobot `gripper` channel
- physical `gripper` -> LeRobot `wrist_roll` channel

Use `python scripts/live_joint_deltas.py` to inspect the current mapping live.

Likely repair path:

1. Use LeRobot's motor setup process to assign the physical wrist-roll motor to ID 5 and physical gripper motor to ID 6.
2. Recalibrate `ladon`.
3. Rerun `python scripts/live_joint_deltas.py`.
4. Set `JOINT_CHANNELS` back to identity in `ladon_config.py` only after the mapping is correct.
