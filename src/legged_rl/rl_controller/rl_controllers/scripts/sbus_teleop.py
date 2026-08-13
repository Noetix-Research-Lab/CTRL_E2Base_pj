#!/usr/bin/env python3
"""Backward-compatible entry point; use sbus_teleop_node.py for new launch files."""

from rl_controllers_teleop.ros_teleop import main


if __name__ == "__main__":
    main()
