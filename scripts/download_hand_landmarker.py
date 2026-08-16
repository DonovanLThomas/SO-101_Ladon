#!/usr/bin/env python
"""Download the MediaPipe hand landmarker model used by hand_follow.py."""

from __future__ import annotations

import argparse
import urllib.request
from pathlib import Path


DEFAULT_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/1/hand_landmarker.task"
)
DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "models" / "hand_landmarker.task"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--force", action="store_true", help="Download even if the output file already exists.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output = args.output.expanduser()
    if output.exists() and not args.force:
        print(f"Model already exists: {output}")
        return

    output.parent.mkdir(parents=True, exist_ok=True)
    print(f"Downloading {args.url}")
    print(f"Saving to {output}")
    urllib.request.urlretrieve(args.url, output)
    print("Done.")


if __name__ == "__main__":
    main()
