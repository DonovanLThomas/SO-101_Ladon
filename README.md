# SO-101 Ladon

Real-time hand-gesture teleoperation for a LeRobot SO-101 follower arm running
on a Jetson Orin Nano.

This repository contains small, safety-focused control scripts for the SO-101
arm named `ladon`. The main demo uses a Raspberry Pi Camera v2 on the Jetson CSI
camera port, MediaPipe hand landmarks, OpenCV frame processing, and LeRobot
motor commands to control the arm from live hand motion.

## What It Does

`scripts/hand_follow.py` tracks one hand and maps the hand position into robot
motion:

- Hand left/right controls `shoulder_pan`.
- Hand up moves `elbow_flex` in the negative direction.
- Hand down moves `elbow_flex` in the positive direction.
- Holding an open palm triggers a friendly lift-and-wave greeting.
- The script smooths targets, limits per-frame motion, clamps to recorded safe
  joint limits, and returns controlled joints to the measured start pose on
  shutdown.

```text
Jetson CSI camera
    -> OpenCV frames
    -> MediaPipe hand landmarks
    -> gesture and target filtering
    -> safe LeRobot SO-101 follower commands
```

## Project Highlights

- Jetson CSI camera support through `nvarguscamerasrc`, useful when Conda
  OpenCV cannot open the Pi Camera v2 directly.
- Browser-based MJPEG preview for SSH development from a Mac or laptop.
- Dry-run mode for validating camera tracking before moving hardware.
- Recorded safe joint limits in `config/safe_joint_limits.json`.
- Shared robot configuration and safety helpers in `ladon_config.py`.
- Utility scripts for pose inspection, joint mapping, safe-limit recording, and
  scripted lift-and-wave tests.

## Repository Layout

```text
.
|-- ladon_config.py                    # Shared robot config, remap, and safety helpers
|-- config/safe_joint_limits.json      # Recorded safe motion envelope for Ladon
|-- docs/joint_mapping_ladon.md        # Current physical-to-LeRobot joint mapping notes
`-- scripts/
    |-- hand_follow.py                 # Vision-guided teleoperation demo
    |-- check_hand_follow_deps.py      # Runtime dependency check
    |-- download_hand_landmarker.py    # MediaPipe model downloader
    |-- read_pose.py                   # Print current robot joint positions
    |-- live_joint_deltas.py           # Inspect joint labels while moving by hand
    |-- record_safe_limits.py          # Record safe limits for all joints
    |-- record_one_safe_limit.py       # Re-record one joint's safe limits
    `-- lift_and_wave.py               # Scripted lift, wave, and return motion
```

## Hardware And Environment

This repo currently assumes:

- Jetson Orin Nano
- Raspberry Pi Camera v2 connected to Jetson `camera0`
- LeRobot checkout at `~/lerobot`
- Conda environment named `lerobot`
- SO-101 follower arm only, no leader arm
- Robot port `/dev/ttyACM0`
- Robot id `ladon`
- Existing LeRobot calibration in the local Hugging Face cache

If the port, robot id, or joint channel mapping changes, update
`ladon_config.py`.

## Safety

Before running any script that moves the robot:

1. Keep one hand near robot power.
2. Start with the arm in a rested, non-stressed pose.
3. Keep cables, fingers, tools, and table edges out of the motion path.
4. Run `python scripts/read_pose.py` before motion.
5. Use `python scripts/live_joint_deltas.py` if a joint label or direction
   seems unclear.

Motion scripts read the current pose first, command relative movements from that
measured pose, clamp targets to recorded safe limits, and limit per-step target
changes.

## Setup

Activate the environment and enter the repo:

```bash
conda activate lerobot
cd ~/lerobot/SO-101_Ladon
```

Check that LeRobot imports:

```bash
python -c "from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig; print('ok')"
```

Install MediaPipe if needed:

```bash
python -m pip install mediapipe
```

Download the MediaPipe hand landmarker model:

```bash
python scripts/download_hand_landmarker.py
```

Verify the hand-follow runtime dependencies:

```bash
python scripts/check_hand_follow_deps.py
```

If the default MediaPipe wheel is unavailable for the Jetson Python version or
architecture, use a Jetson-compatible MediaPipe wheel.

## Camera Check

The Pi Camera v2 should appear as Jetson camera `sensor-id=0`:

```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=100 ! \
'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' ! \
fakesink
```

For the repo demo, use the Jetson CSI backend:

```bash
python -u scripts/hand_follow.py \
  --dry-run \
  --camera 0 \
  --camera-backend jetson-csi \
  --pan-range-deg 20 \
  --elbow-range-deg 25 \
  --no-preview
```

## Preview From A Mac Over SSH

From the Mac, open an SSH tunnel to the Jetson:

```bash
ssh -L 8080:127.0.0.1:8080 dontech@<jetson-host-or-ip>
```

Inside that SSH session on the Jetson:

```bash
conda activate lerobot
cd ~/lerobot/SO-101_Ladon

python -u scripts/hand_follow.py \
  --dry-run \
  --camera 0 \
  --camera-backend jetson-csi \
  --pan-range-deg 20 \
  --elbow-range-deg 25 \
  --no-preview \
  --preview-stream-port 8080
```

Open this URL on the Mac:

```text
http://127.0.0.1:8080/stream.mjpg
```

## Run Live Teleoperation

Start with a conservative live test:

```bash
python -u scripts/hand_follow.py \
  --camera 0 \
  --camera-backend jetson-csi \
  --pan-range-deg 20 \
  --elbow-range-deg 25 \
  --fps 10 \
  --wave-joint wrist_flex \
  --no-preview \
  --preview-stream-port 8080
```

Useful tuning flags:

- `--pan-range-deg`: maximum `shoulder_pan` offset from the measured start pose.
- `--elbow-range-deg`: maximum `elbow_flex` offset from the measured start pose.
- `--smoothing`: low-pass factor for target changes.
- `--max-step-deg`: maximum joint target change per control tick.
- `--wave-amplitude`: wrist wave size during the greeting. Hand-follow defaults
  to 6 degrees and clamps at 8 degrees.
- `--greeting-lift-deg`: shoulder lift offset before the open-palm wave.
- `--greeting-wrist-offset-deg`: wrist presentation offset before the
  open-palm wave. It now has its own 45 degree clamp so larger wrist
  presentation values are not limited by the shoulder lift clamp.
- `--greeting-settle-seconds`: time to ease into and out of the greeting pose.
- `--no-hand-behavior hold`: hold the last target when no hand is visible.
- `--no-hand-behavior return`: drift back to the measured start pose when no
  hand is visible.

To increase motion, raise ranges gradually:

```bash
--pan-range-deg 30 --elbow-range-deg 35
```

If the script reports a target outside recorded safe limits, it is clamping the
command to protect the arm.

## Other Robot Utilities

Read the current robot pose:

```bash
python scripts/read_pose.py
```

Monitor live joint deltas while moving the arm by hand:

```bash
python scripts/live_joint_deltas.py
```

Run a scripted lift, wave, and return:

```bash
python scripts/lift_and_wave.py \
  --shoulder-lift-offset 10 \
  --elbow-flex-offset -45 \
  --wrist-flex-offset -5 \
  --amplitude 2 \
  --lift-seconds 6
```

Record safe limits for all joints:

```bash
python scripts/record_safe_limits.py
```

Re-record one joint:

```bash
python scripts/record_one_safe_limit.py --joint elbow_flex
```

## Joint Mapping Note

Ladon's physical `wrist_roll` and physical `gripper` are currently swapped
relative to the standard SO-101 follower mapping. `ladon_config.py` includes a
temporary software remap so scripts can use physical joint names consistently.

See `docs/joint_mapping_ladon.md` for the observed mapping and repair path.

## Implementation Notes

- Arm joints are commanded in degrees because `use_degrees=True`.
- Downloaded MediaPipe model files are ignored with `models/*.task`.
- LeRobot's internal relative target clamp is disabled in this repo; motion
  scripts instead use recorded absolute safe limits plus per-step target limits.
- The Jetson CSI backend captures a supported sensor mode and scales frames to
  the processing resolution requested by `--width` and `--height`.
