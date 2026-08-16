#!/usr/bin/env python
"""Check runtime dependencies for the hand-follow demo."""

from __future__ import annotations

import importlib.util
import importlib
import os
import sys
from pathlib import Path


REQUIRED_MODULES = ["cv2", "numpy", "mediapipe", "lerobot"]
DEFAULT_MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "hand_landmarker.task"


def main() -> None:
    missing = []
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
    print("Hand-follow dependency check")
    print("-" * 32)
    for module in REQUIRED_MODULES:
        if importlib.util.find_spec(module) is None:
            print(f"{module:12s} missing")
            missing.append(module)
            continue

        try:
            importlib.import_module(module)
            status = "ok"
        except Exception as exc:  # noqa: BLE001
            status = f"import failed: {exc}"
            missing.append(module)
        print(f"{module:12s} {status}")

    if missing:
        print("\nMissing modules:", ", ".join(missing))
        print("Activate the lerobot conda env, then install the missing packages.")
        print("For MediaPipe on Jetson, prefer a Jetson-compatible wheel if the default pip wheel fails.")
        raise SystemExit(1)

    if DEFAULT_MODEL_PATH.exists():
        print(f"model        ok ({DEFAULT_MODEL_PATH})")
    else:
        print(f"model        missing ({DEFAULT_MODEL_PATH})")
        print("\nDownload the model with:")
        print("python scripts/download_hand_landmarker.py")
        raise SystemExit(1)

    print("\nAll required modules are importable.")


if __name__ == "__main__":
    sys.exit(main())
