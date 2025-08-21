#!/usr/bin/env python3
"""
UR3e - Fun Demo Routine
Sequence: 360 spin → dab pose → home → bye-bye wave.
"""

import time

import rclpy

from robot_setup import setup_robot_workspace
from ur3_types import JointPositions as Jpos
from ur3e_lib import UR3eController

home = Jpos([0, -90, 0, -90, 0, 0])
bow = Jpos([0, -45, -100, -30, 0, 0])


def main():
    rclpy.init()
    robot = UR3eController("ur3e_demo_node")

    # Setup robot workspace with collision objects
    workspace = setup_robot_workspace(robot)

    # Use safer positions that account for collision objects
    safe_positions = workspace.get_safe_demo_positions()
    home = Jpos(safe_positions["home"])
    bow = Jpos(safe_positions["forward_safe"])

    try:
        # Go to home locations
        robot.move_to_joint_positions(home)
        # time.sleep(1)

        # Bow
        robot.move_to_joint_positions(bow)
        time.sleep(1)

        # Bye-bye wave
        hand = bow.copy()
        for angle in [-60, 0, -30]:
            hand.wrist1 = angle
            robot.move_to_joint_positions(hand)

        # Go to home locations
        robot.move_to_joint_positions(home)

        print("Demo finished!")

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        robot.destroy_node()
        rclpy.shutdown()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()
