#!/usr/bin/env python3
"""
UR3 Robot Setup Configuration
Defines the robot stand, tool, and workspace collision objects
"""


class RobotSetup:
    """Configuration for robot collision objects and workspace setup"""

    def __init__(self):
        # Robot stand - vertical 20x20cm aluminum extrusion column
        self.robot_stand = {
            "name": "robot_stand",
            "pose": [0.0, 0.0, -0.3, 0.0, 0.0, 0.0, 1.0],  # Center of column (30cm below robot base)
            "dimensions": [0.2, 0.2, 0.6],  # 20x20x60cm vertical column
            "frame_id": "base_link",
        }

        # Control box - gray UR control box attached to the back of the robot stand base
        self.control_box = {
            "name": "control_box",
            "pose": [0.0, -0.2, -0.2, 0.0, 0.0, 0.0, 1.0],  # 20cm back from robot (attached to stand), 20cm below
            "dimensions": [0.5, 0.2, 0.25],  # 50x20x25cm control box
            "frame_id": "base_link",
        }

        # Tool/gripper attached to robot (from photos)
        self.robot_tool = {
            "name": "gripper_tool",
            "dimensions": [0.08, 0.06, 0.12],  # Gripper dimensions from photos
            "link_name": "tool0",
            "pose_offset": [0.0, 0.0, 0.06, 0.0, 0.0, 0.0, 1.0],  # 6cm forward from tool0
        }

    def apply_basic_setup(self, robot_controller):
        """
        Apply basic robot setup (stand, control box, tool)

        Args:
            robot_controller: UR3eController instance

        Returns:
            bool: True if all objects added successfully

        """
        success = True

        # Add robot stand
        if not robot_controller.add_box_collision_object(
            self.robot_stand["name"],
            self.robot_stand["pose"],
            self.robot_stand["dimensions"],
            self.robot_stand["frame_id"],
        ):
            success = False

        # Add control box
        if not robot_controller.add_box_collision_object(
            self.control_box["name"],
            self.control_box["pose"],
            self.control_box["dimensions"],
            self.control_box["frame_id"],
        ):
            success = False

        # Attach robot tool
        if not robot_controller.attach_tool_collision_object(
            self.robot_tool["name"],
            self.robot_tool["dimensions"],
            self.robot_tool["link_name"],
            self.robot_tool["pose_offset"],
        ):
            success = False

        return success

    def add_custom_box(self, robot_controller, name, pose, dimensions, frame_id="base_link"):
        """
        Add a custom collision box to the workspace

        Args:
            robot_controller: UR3eController instance
            name (str): Unique name for the box
            pose (list): [x, y, z, qx, qy, qz, qw] position and orientation
            dimensions (list): [length, width, height] in meters
            frame_id (str): Reference frame

        Returns:
            bool: True if successfully added

        """
        return robot_controller.add_box_collision_object(name, pose, dimensions, frame_id)

    def remove_all_objects(self, robot_controller):
        """
        Remove all collision objects from planning scene

        Args:
            robot_controller: UR3eController instance

        """
        # Remove basic setup objects
        robot_controller.remove_collision_object(self.robot_stand["name"])
        robot_controller.remove_collision_object(self.control_box["name"])

    def get_safe_demo_positions(self):
        """
        Get safe joint positions for demo that avoid collisions

        Returns:
            dict: Dictionary of safe positions considering collision objects

        """
        return {
            "home": [0, -90, 0, -90, 0, 0],
            "raised": [0, -60, -30, -120, 0, 0],  # Higher to avoid stand
            "left_side": [90, -80, -20, -110, 0, 0],  # Away from control box
            "right_side": [-90, -80, -20, -110, 0, 0],
            "forward_safe": [0, -45, -60, -105, 0, 0],  # Forward safe position
        }


# Convenience function for quick setup
def setup_robot_workspace(robot_controller):
    """
    Quick setup function to configure robot workspace
    Cleans up any existing objects first, then sets up the workspace

    Args:
        robot_controller: UR3eController instance

    Returns:
        RobotSetup: Setup instance for further customization

    """
    setup = RobotSetup()

    robot_controller.get_logger().info("Cleaning up previous collision objects...")
    setup.remove_all_objects(robot_controller)

    robot_controller.get_logger().info("Setting up robot workspace collision objects...")

    if setup.apply_basic_setup(robot_controller):
        robot_controller.get_logger().info("✓ Basic workspace setup completed")
    else:
        robot_controller.get_logger().warn("⚠ Some basic objects failed to load")

    return setup
