#!/usr/bin/env python
"""Print the current measured joint positions for Ladon."""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ladon_config import make_ladon, pose_from_observation, print_pose


def main() -> None:
    robot = make_ladon()
    robot.connect()
    try:
        pose = pose_from_observation(robot.get_observation())
        print_pose(pose)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
