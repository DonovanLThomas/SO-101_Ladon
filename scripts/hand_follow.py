#!/usr/bin/env python
"""Control Ladon's shoulder pan from hand position and wave on open palm."""

from __future__ import annotations

import argparse
import http.server
import os
import socketserver
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ladon_config import (  # noqa: E402
    DEFAULT_FPS,
    DEFAULT_WAVE_AMPLITUDE_DEG,
    DEFAULT_WAVE_CYCLES,
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
from scripts.lift_and_wave import wave_from_pose  # noqa: E402


PAN_JOINT = "shoulder_pan"
ELBOW_JOINT = "elbow_flex"
CONTROL_JOINTS = [PAN_JOINT, ELBOW_JOINT]
RETURN_JOINTS = [PAN_JOINT, ELBOW_JOINT]
LONG_FINGER_TIPS = [8, 12, 16, 20]
LONG_FINGER_PIPS = [6, 10, 14, 18]
DEFAULT_MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "hand_landmarker.task"


@dataclass
class HandState:
    seen: bool
    center_x: float = 0.5
    center_y: float = 0.5
    open_palm: bool = False
    extended_fingers: int = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default="0", help="OpenCV camera index or device path, e.g. 0 or /dev/video0.")
    parser.add_argument(
        "--camera-backend",
        choices=["auto", "opencv", "jetson-csi"],
        default="auto",
        help="Camera reader to use. auto falls back to Jetson CSI when OpenCV cannot open a numeric camera.",
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH, help="MediaPipe hand landmarker .task file.")
    parser.add_argument("--dry-run", action="store_true", help="Run vision and print targets without moving the robot.")
    parser.add_argument("--pan-range-deg", type=float, default=20.0, help="Max shoulder_pan offset from start pose.")
    parser.add_argument(
        "--elbow-range-deg",
        type=float,
        default=25.0,
        help="Max elbow_flex offset from start pose. Hand up moves elbow_flex negative.",
    )
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS)
    parser.add_argument("--no-preview", action="store_true", help="Disable OpenCV preview window for SSH/headless use.")
    parser.add_argument(
        "--preview-stream-port",
        type=int,
        default=0,
        help="Serve the annotated preview as MJPEG on this local port, useful with an SSH tunnel.",
    )
    parser.add_argument("--preview-stream-host", default="127.0.0.1", help="Host interface for --preview-stream-port.")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after this many frames. 0 runs until interrupted.")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--sensor-width", type=int, default=1280, help="Jetson CSI capture width before scaling.")
    parser.add_argument("--sensor-height", type=int, default=720, help="Jetson CSI capture height before scaling.")
    parser.add_argument("--sensor-fps", type=int, default=60, help="Jetson CSI capture framerate before throttling.")
    parser.add_argument("--wave-joint", choices=["wrist_flex", "shoulder_pan"], default="wrist_flex")
    parser.add_argument("--wave-amplitude", type=float, default=DEFAULT_WAVE_AMPLITUDE_DEG)
    parser.add_argument("--wave-cycles", type=int, default=DEFAULT_WAVE_CYCLES)
    parser.add_argument("--wave-cycle-seconds", type=float, default=1.6)
    parser.add_argument("--wave-cooldown-seconds", type=float, default=3.0)
    parser.add_argument("--open-palm-hold-seconds", type=float, default=0.6)
    parser.add_argument(
        "--no-hand-behavior",
        choices=["hold", "return"],
        default="hold",
        help="Hold last target or slowly return to the measured start pose when no hand is visible.",
    )
    parser.add_argument("--smoothing", type=float, default=0.25, help="0..1 low-pass factor for hand target changes.")
    parser.add_argument("--max-step-deg", type=float, default=1.0, help="Max target change per control tick.")
    parser.add_argument("--min-detection-confidence", type=float, default=0.6)
    parser.add_argument("--min-tracking-confidence", type=float, default=0.5)
    return parser.parse_args()


def camera_source(value: str) -> int | str:
    return int(value) if value.isdigit() else value


class OpenCVCamera:
    def __init__(self, cv2, source: int | str, width: int, height: int, fps: float) -> None:
        self.cap = cv2.VideoCapture(source)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

    def is_opened(self) -> bool:
        return self.cap.isOpened()

    def read(self):
        return self.cap.read()

    def release(self) -> None:
        self.cap.release()


class JetsonCsiCamera:
    def __init__(
        self,
        np,
        sensor_id: int,
        width: int,
        height: int,
        sensor_width: int,
        sensor_height: int,
        sensor_fps: int,
    ) -> None:
        self.np = np
        self.width = width
        self.height = height
        self.frame_bytes = width * height * 3
        cmd = [
            "gst-launch-1.0",
            "-q",
            "nvarguscamerasrc",
            f"sensor-id={sensor_id}",
            "!",
            f"video/x-raw(memory:NVMM),width={sensor_width},height={sensor_height},framerate={sensor_fps}/1",
            "!",
            "nvvidconv",
            "!",
            f"video/x-raw,width={width},height={height},format=BGRx",
            "!",
            "videoconvert",
            "!",
            "video/x-raw,format=BGR",
            "!",
            "fdsink",
            "fd=1",
            "sync=false",
        ]
        self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

    def is_opened(self) -> bool:
        return self.process.poll() is None and self.process.stdout is not None

    def read(self):
        if self.process.stdout is None:
            return False, None
        data = self.process.stdout.read(self.frame_bytes)
        if len(data) != self.frame_bytes:
            return False, None
        frame = self.np.frombuffer(data, dtype=self.np.uint8).reshape((self.height, self.width, 3)).copy()
        return True, frame

    def release(self) -> None:
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2.0)


class PreviewState:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.jpeg: bytes | None = None

    def update(self, jpeg: bytes) -> None:
        with self.condition:
            self.jpeg = jpeg
            self.condition.notify_all()


class ThreadingHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def make_preview_handler(state: PreviewState):
    class PreviewHandler(http.server.BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            if self.path not in ("/", "/stream.mjpg"):
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last_jpeg = None
            while True:
                with state.condition:
                    state.condition.wait_for(lambda: state.jpeg is not None and state.jpeg is not last_jpeg)
                    last_jpeg = state.jpeg
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(last_jpeg)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(last_jpeg)
                    self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    break

        def log_message(self, format: str, *args) -> None:
            return

    return PreviewHandler


def start_preview_server(host: str, port: int) -> tuple[PreviewState, ThreadingHTTPServer]:
    state = PreviewState()
    server = ThreadingHTTPServer((host, port), make_preview_handler(state))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"Preview stream: http://{host}:{server.server_port}/stream.mjpg")
    return state, server


def open_camera(args: argparse.Namespace, cv2, np, fps: float):
    source = camera_source(args.camera)
    if args.camera_backend in ("auto", "opencv"):
        cap = OpenCVCamera(cv2, source, args.width, args.height, fps)
        if cap.is_opened():
            return cap, "opencv"
        cap.release()
        if args.camera_backend == "opencv":
            raise SystemExit(f"Could not open camera {args.camera}. Try `lerobot-find-cameras opencv`.")

    if not args.camera.isdigit():
        raise SystemExit(
            f"Could not open camera {args.camera} with OpenCV, and Jetson CSI requires a numeric sensor id."
        )

    cap = JetsonCsiCamera(
        np,
        int(args.camera),
        args.width,
        args.height,
        args.sensor_width,
        args.sensor_height,
        args.sensor_fps,
    )
    if not cap.is_opened():
        cap.release()
        raise SystemExit(f"Could not open Jetson CSI camera sensor-id={args.camera}.")
    return cap, "jetson-csi"


def clamp(value: float, minimum: float, maximum: float) -> float:
    return min(max(value, minimum), maximum)


def import_vision_stack():
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
    try:
        import cv2
        import mediapipe as mp
        import numpy as np
        from mediapipe.tasks.python import vision
        from mediapipe.tasks.python.core import base_options
        from mediapipe.tasks.python.vision.core import vision_task_running_mode
    except ImportError as exc:
        missing = exc.name or "a vision dependency"
        raise SystemExit(
            f"Missing {missing}. Run `conda activate lerobot`, then "
            "`python scripts/check_hand_follow_deps.py`. Install MediaPipe before running this demo."
        ) from exc
    return cv2, mp, np, vision, base_options, vision_task_running_mode


def detect_hand(results) -> HandState:
    if not results.hand_landmarks:
        return HandState(seen=False)

    landmarks = results.hand_landmarks[0]
    center_x = sum(point.x for point in landmarks) / len(landmarks)
    center_y = sum(point.y for point in landmarks) / len(landmarks)
    extended = sum(1 for tip, pip in zip(LONG_FINGER_TIPS, LONG_FINGER_PIPS) if landmarks[tip].y < landmarks[pip].y)
    return HandState(
        seen=True,
        center_x=clamp(center_x, 0.0, 1.0),
        center_y=clamp(center_y, 0.0, 1.0),
        open_palm=extended >= 4,
        extended_fingers=extended,
    )


def target_pan_from_hand(start_pan: float, hand: HandState, pan_range_deg: float) -> float:
    centered_x = (hand.center_x - 0.5) * 2.0
    return start_pan + centered_x * pan_range_deg


def target_elbow_from_hand(start_elbow: float, hand: HandState, elbow_range_deg: float) -> float:
    centered_y = (hand.center_y - 0.5) * 2.0
    return start_elbow + centered_y * elbow_range_deg


def step_toward(current: float, target: float, max_step: float) -> float:
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return current + (max_step if delta > 0 else -max_step)


def draw_overlay(
    cv2,
    vision,
    frame,
    hand_landmarks,
    state: HandState,
    target_pan: float,
    target_elbow: float,
    dry_run: bool,
) -> None:
    height, width = frame.shape[:2]
    for landmarks in hand_landmarks:
        points = [(int(point.x * width), int(point.y * height)) for point in landmarks]
        for connection in vision.HandLandmarksConnections.HAND_CONNECTIONS:
            cv2.line(frame, points[connection.start], points[connection.end], (80, 220, 120), 2)
        for x, y in points:
            cv2.circle(frame, (x, y), 3, (0, 255, 255), -1)

    mode = "DRY RUN" if dry_run else "ROBOT LIVE"
    seen = "hand" if state.seen else "no hand"
    palm = "open palm" if state.open_palm else "tracking"
    cv2.putText(frame, mode, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
    cv2.putText(
        frame,
        f"{seen} | {palm} | pan {target_pan:6.2f} | elbow {target_elbow:6.2f}",
        (12, 58),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 0),
        2,
    )
    cv2.putText(frame, "q or Esc quits", (12, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)


def make_wave_args(args: argparse.Namespace) -> SimpleNamespace:
    return SimpleNamespace(
        wave_joint=args.wave_joint,
        amplitude=clamp_abs(args.wave_amplitude, MAX_WAVE_AMPLITUDE_DEG, "wave-amplitude"),
        shoulder_assist=0.0,
        cycles=max(1, args.wave_cycles),
        cycle_seconds=max(0.5, args.wave_cycle_seconds),
        fps=args.fps,
    )


def maybe_send_pose(robot, pose: dict[str, float], dry_run: bool) -> None:
    if dry_run:
        return
    robot.send_action(action_from_pose(pose, CONTROL_JOINTS))


def main() -> None:
    args = parse_args()
    cv2, mp, np, vision, base_options, running_mode = import_vision_stack()

    model_path = args.model.expanduser()
    if not model_path.exists():
        raise SystemExit(
            f"Missing MediaPipe model: {model_path}\n"
            "Run `python scripts/download_hand_landmarker.py`, or pass --model /path/to/hand_landmarker.task."
        )

    fps = max(args.fps, 1.0)
    pan_range = abs(clamp_abs(args.pan_range_deg, 60.0, "pan-range-deg"))
    elbow_range = abs(clamp_abs(args.elbow_range_deg, 45.0, "elbow-range-deg"))
    smoothing = clamp(args.smoothing, 0.0, 1.0)
    max_step = abs(clamp_abs(args.max_step_deg, 5.0, "max-step-deg"))
    require_motion_joint_allowed(PAN_JOINT)
    require_motion_joint_allowed(ELBOW_JOINT)
    require_motion_joint_allowed(args.wave_joint)

    cap, camera_backend = open_camera(args, cv2, np, fps)
    print(f"Camera opened with {camera_backend} backend.")
    preview = None
    preview_server = None
    if args.preview_stream_port:
        preview, preview_server = start_preview_server(args.preview_stream_host, args.preview_stream_port)

    robot = None
    start_pose = {PAN_JOINT: 0.0, "wrist_flex": 0.0, "shoulder_lift": 0.0, "elbow_flex": 0.0}
    if not args.dry_run:
        robot = make_ladon()
        robot.connect()
        start_pose = pose_from_observation(robot.get_observation())
        print("Measured starting pose. The script will return shoulder_pan here before disconnecting:")
        print_pose(start_pose)
    else:
        print("Dry run: camera and gesture logic only. Robot will not move.")

    target_pan = start_pose[PAN_JOINT]
    target_elbow = start_pose[ELBOW_JOINT]
    filtered_pan = target_pan
    filtered_elbow = target_elbow
    open_since: float | None = None
    next_wave_time = 0.0
    wave_args = make_wave_args(args)

    options = vision.HandLandmarkerOptions(
        base_options=base_options.BaseOptions(model_asset_path=str(model_path)),
        running_mode=running_mode.VisionTaskRunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=args.min_detection_confidence,
        min_hand_presence_confidence=args.min_detection_confidence,
        min_tracking_confidence=args.min_tracking_confidence,
    )
    hands = vision.HandLandmarker.create_from_options(options)

    try:
        frame_count = 0
        while True:
            loop_start = time.perf_counter()
            ok, frame = cap.read()
            if not ok:
                print("Camera frame read failed; stopping.")
                break
            frame_count += 1

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            results = hands.detect_for_video(mp_image, int(time.monotonic() * 1000))
            state = detect_hand(results)

            if state.seen:
                desired_pan = target_pan_from_hand(start_pose[PAN_JOINT], state, pan_range)
                desired_elbow = target_elbow_from_hand(start_pose[ELBOW_JOINT], state, elbow_range)
            elif args.no_hand_behavior == "return":
                desired_pan = start_pose[PAN_JOINT]
                desired_elbow = start_pose[ELBOW_JOINT]
            else:
                desired_pan = target_pan
                desired_elbow = target_elbow

            smoothed_pan = filtered_pan + smoothing * (desired_pan - filtered_pan)
            smoothed_elbow = filtered_elbow + smoothing * (desired_elbow - filtered_elbow)
            filtered_pan = step_toward(filtered_pan, smoothed_pan, max_step)
            filtered_elbow = step_toward(filtered_elbow, smoothed_elbow, max_step)
            target_pan = filtered_pan
            target_elbow = filtered_elbow
            command_pose = {PAN_JOINT: target_pan, ELBOW_JOINT: target_elbow}
            maybe_send_pose(robot, command_pose, args.dry_run)

            now = time.perf_counter()
            if state.open_palm:
                open_since = now if open_since is None else open_since
                held_open = now - open_since >= args.open_palm_hold_seconds
                if held_open and now >= next_wave_time:
                    if args.dry_run:
                        print(f"Wave trigger: {args.wave_joint} from open palm.")
                    else:
                        wave_pose = dict(start_pose)
                        wave_pose[PAN_JOINT] = target_pan
                        wave_pose[ELBOW_JOINT] = target_elbow
                        wave_from_pose(robot, wave_pose, wave_args)
                    next_wave_time = time.perf_counter() + args.wave_cooldown_seconds
                    open_since = None
            else:
                open_since = None

            if not args.no_preview:
                draw_overlay(cv2, vision, frame, results.hand_landmarks, state, target_pan, target_elbow, args.dry_run)
                cv2.imshow("Ladon Hand Follow", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
            elif preview is not None:
                draw_overlay(cv2, vision, frame, results.hand_landmarks, state, target_pan, target_elbow, args.dry_run)

            if preview is not None:
                ok, jpeg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if ok:
                    preview.update(jpeg.tobytes())

            if args.no_preview and int(now * 2) != int((now - 1.0 / fps) * 2):
                hand_label = "open" if state.open_palm else "seen" if state.seen else "none"
                print(
                    f"hand={hand_label:5s} "
                    f"target_{PAN_JOINT}={target_pan:7.2f} "
                    f"target_{ELBOW_JOINT}={target_elbow:7.2f}"
                )

            if args.max_frames and frame_count >= args.max_frames:
                break

            sleep_step(loop_start, fps)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        hands.close()
        cap.release()
        if preview_server is not None:
            preview_server.shutdown()
            preview_server.server_close()
        if not args.no_preview:
            cv2.destroyAllWindows()
        if robot is not None:
            try:
                current_pose = pose_from_observation(robot.get_observation())
                print(f"Returning {PAN_JOINT} to measured start pose.")
                slew_to_pose(robot, current_pose, start_pose, joints=RETURN_JOINTS, seconds=2.0, fps=fps)
            finally:
                robot.disconnect()


if __name__ == "__main__":
    main()
