#!/usr/bin/env python3
"""
UR3e - Fun Demo Routine
Sequence: 360 spin → dab pose → home → bye-bye wave.
"""

import rclpy
import time
from ur3e_lib import UR3eController


def main():
    rclpy.init()
    robot = UR3eController("ur3e_demo_node")

    try:
        home = [0, -90, 0, -90, 0, 0]

        # Step 1: 360 Spin
        print("🌀 Full spin...")
        spin_pose = home.copy()
        spin_pose[0] = 360  # rotate right
        robot.move_to_joint_positions(spin_pose)
        time.sleep(1)

        # Step 2: Dab Pose
        print("🤷 Dab pose...")
        dab_pose = home.copy()
        dab_pose[1] = -120  # shoulder down
        dab_pose[2] = 70  # elbow bent up
        dab_pose[3] = -60  # wrist1 twist
        dab_pose[5] = 45  # wrist3 angle
        robot.move_to_joint_positions(dab_pose)
        time.sleep(1.5)

        # Step 3: Return home
        print("➡️ Returning home...")
        robot.move_to_joint_positions(home)
        time.sleep(0.8)

        # Step 4: Bye-bye wave (from home)
        print("👋 Bye-bye wave...")
        wave_pose = home.copy()
        for angle in [-30, 30, -30, 30]:
            wave_pose[5] = angle
            robot.move_to_joint_positions(wave_pose)
            time.sleep(0.4)

        # Final: Back to exact home
        robot.move_to_joint_positions(home)
        time.sleep(0.5)

        print("✅ Demo finished!")

    except KeyboardInterrupt:
        print("\n❌ Interrupted by user")

    finally:
        robot.destroy_node()
        rclpy.shutdown()
        print("🔻 Shutdown complete.")


if __name__ == "__main__":
    main()
