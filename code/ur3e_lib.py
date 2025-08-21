#!/usr/bin/env python3
"""
UR3e Robot Control Library - Joint Control Only

A clean library for controlling UR3e robot with joint positions (degrees).
No Cartesian control to avoid complications.

Usage:
    from ur3e_lib import UR3eController
    from ur3_types import JointPositions

    robot = UR3eController()

    # Using JointPositions type
    joints = JointPositions([0, -90, 20, -90, 0, 0])
    robot.move_to_joint_positions(joints)

    # Or modify individual joints
    joints.base = 45
    robot.move_to_joint_positions(joints)

    robot.go_home()
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
    CollisionObject,
    AttachedCollisionObject,
    PlanningScene,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion
import math
import time
from ur3_types import JointPositions


class UR3eController(Node):
    """
    UR3e Robot Controller Library - Joint Control Only

    Provides interface for joint control with degrees.
    """

    def __init__(self, node_name="ur3e_controller"):
        super().__init__(node_name)

        # Create action client for trajectory control
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        )

        # Create action client for MoveGroup (safe planning)
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        
        # Create service client for getting planning scene (real robot state)
        self.get_planning_scene_client = self.create_client(GetPlanningScene, "/get_planning_scene")
        
        # Create service client for applying planning scene changes (collision objects)
        self.apply_planning_scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState, "joint_states", self._joint_state_callback, 10
        )

        # Robot state
        self.current_joint_state = None
        self.joint_state_received = False

        # UR3e joint names (in order)
        self.ur_joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Home position in degrees
        self.home_position = JointPositions([0.0, -90.0, 0.0, -90.0, 0.0, 0.0])

        # Initialize connection
        self._initialize_connection()

    def _initialize_connection(self):
        """Initialize connection to ROS 2 services"""
        self.get_logger().info("Initializing UR3e controller...")

        # Wait for trajectory controller
        self.get_logger().info("Connecting to trajectory controller...")
        if not self.trajectory_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().fatal("Trajectory controller not available")
            self.get_logger().fatal("   • Make sure the robot is powered on")
            self.get_logger().fatal("   • Ensure UR driver is running: ur3 run driver")
            self.get_logger().fatal("   • On the robot teach pendant, load your external_control.urp program")
            self.get_logger().fatal("   • Press the ▶ Play button on the teach pendant to start External Control")
            self.get_logger().fatal("   • Check network connection: ping 192.168.1.102")
            sys.exit(1)

        # Wait for MoveGroup
        self.get_logger().info("Connecting to MoveGroup for safe planning...")
        if not self.move_group_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().fatal("MoveGroup not available")
            sys.exit(1)

        # Wait for joint states
        self.get_logger().info("Waiting for joint states...")
        while not self.joint_state_received:
            rclpy.spin_once(self)
            time.sleep(0.1)

        self.get_logger().info("UR3e controller ready!")

    def _joint_state_callback(self, msg):
        """Callback for joint state messages"""
        self.current_joint_state = msg
        self.joint_state_received = True

    def _goal_feedback_callback(self, feedback_msg):
        """Callback for trajectory execution feedback"""
        pass

    def degrees_to_radians(self, degrees_list):
        """Convert list of degrees to radians"""
        return [math.radians(deg) for deg in degrees_list]

    def radians_to_degrees(self, radians_list):
        """Convert list of radians to degrees"""
        return [math.degrees(rad) for rad in radians_list]

    def get_current_joint_positions(self):
        """
        Get current joint positions in degrees from MoveIt planning scene (real robot state)

        Returns:
            JointPositions: Joint positions object with individual joint values in degrees
            None: If robot state not available or robot not connected
        """
        # Use MoveIt's planning scene to get actual robot state
        if not self.get_planning_scene_client.service_is_ready():
            self.get_logger().error("Planning scene service not available - robot may not be connected")
            return None
            
        try:
            request = GetPlanningScene.Request()
            request.components.components = request.components.ROBOT_STATE
            
            future = self.get_planning_scene_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if not future.done():
                self.get_logger().error("Timeout getting planning scene - robot may not be connected")
                return None
                
            response = future.result()
            robot_state = response.scene.robot_state
            
            # Extract joint positions from robot state
            ur_positions = []
            for joint_name in self.ur_joint_names:
                if joint_name in robot_state.joint_state.name:
                    idx = robot_state.joint_state.name.index(joint_name)
                    ur_positions.append(robot_state.joint_state.position[idx])
                else:
                    self.get_logger().error(f"Joint {joint_name} not found in robot state")
                    return None

            degrees_list = self.radians_to_degrees(ur_positions)
            return JointPositions(degrees_list)
            
        except Exception as e:
            self.get_logger().error(f"Failed to get current robot state: {e}")
            return None

    def move_to_joint_positions(self, target_positions, duration_sec=5.0):
        """
        SAFE Move robot to target joint positions using MoveIt planning with collision checking

        Args:
            target_positions (JointPositions): Joint positions in degrees
            duration_sec (float): Time to complete movement

        Returns:
            bool: True if successful, False otherwise
        """
        if not isinstance(target_positions, JointPositions):
            self.get_logger().error("target_positions must be a JointPositions object")
            return False

        target_positions_deg = target_positions.to_list()

        # Convert to radians
        target_positions_rad = self.degrees_to_radians(target_positions_deg)

        current_positions_deg = self.get_current_joint_positions()
        if not current_positions_deg:
            self.get_logger().error("Could not get current joint positions")
            return False

        current_positions_list = current_positions_deg.to_list()
        self.get_logger().info(
            f"SAFE Planning: {[round(p, 2) for p in current_positions_list]} → {[round(p, 2) for p in target_positions_deg]} degrees"
        )

        # Create MoveGroup goal for safe planning
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = "ur_manipulator"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1

        # Set joint constraints for target position
        goal.request.goal_constraints = [Constraints()]

        for i, joint_name in enumerate(self.ur_joint_names):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = target_positions_rad[i]
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            goal.request.goal_constraints[0].joint_constraints.append(joint_constraint)

        # Set planning options for safety
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False  # Plan and execute
        goal.planning_options.look_around = True  # Enable collision checking
        goal.planning_options.look_around_attempts = 3
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.1

        # Send goal to MoveIt for safe planning
        self.get_logger().info("   Planning motion with collision avoidance...")
        try:
            future = self.move_group_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            if not future.done():
                self.get_logger().error(" TIMEOUT: MoveIt goal submission timed out")
                return False

            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error(
                    " SAFETY: Joint movement goal REJECTED by MoveIt"
                )
                self.get_logger().error("   Possible reasons:")
                self.get_logger().error("   • Position would cause self-collision")
                self.get_logger().error("   • Joint limits exceeded")
                self.get_logger().error("   • Unreachable configuration")
                self.get_logger().error("   • Singularity detected")
                return False

            self.get_logger().info("   Safety check passed, executing motion...")

            # Wait for execution result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

            if not result_future.done():
                self.get_logger().error(" TIMEOUT: MoveIt execution timed out")
                return False

            result = result_future.result()
            if not result or not result.result:
                self.get_logger().error(" FAILED: No result received from MoveIt")
                return False

            success = result.result.error_code.val == 1  # MoveItErrorCodes::SUCCESS

        except Exception as e:
            self.get_logger().error(f" EXCEPTION during MoveIt operation: {e}")
            return False

        if success:
            self.get_logger().info("   SAFE joint movement completed successfully!")
        else:
            error_code = result.result.error_code.val
            self.get_logger().error(
                f" SAFE movement failed with MoveIt error code: {error_code}"
            )

            # Provide helpful error messages
            if error_code == -4:
                self.get_logger().error(
                    " CONTROL_FAILED: Robot hardware communication failed!"
                )
                self.get_logger().error(
                    "   • Check if robot is powered on and connected"
                )
                self.get_logger().error("   • Verify network connection to robot")
                self.get_logger().error("   • Ensure robot is not in protective stop")
                self.get_logger().error("   • Check if UR driver is running properly")
                self.get_logger().fatal("   CRITICAL ERROR: Exiting due to robot communication failure")
                sys.exit(1)
            elif error_code == -21:
                self.get_logger().error(
                    " COLLISION: Target position would cause robot to hit itself!"
                )
            elif error_code == -22:
                self.get_logger().error(" COLLISION: Current position has collision!")
            elif error_code == -11:
                self.get_logger().error(" INVALID: Joint constraints are invalid")
            elif error_code == -2:
                self.get_logger().error(" NO PATH: Could not find safe path to target")
            elif error_code == -7:
                self.get_logger().error(" TIMEOUT: Planning took too long")
            else:
                self.get_logger().error(f" UNKNOWN ERROR: Code {error_code}")

        return success

    def go_home(self, duration_sec=5.0):
        """
        Move robot to home position

        Args:
            duration_sec (float): Time to complete movement

        Returns:
            bool: True if successful, False otherwise
        """
        self.get_logger().info("Moving to home position...")
        return self.move_to_joint_positions(self.home_position, duration_sec)

    def add_box_collision_object(self, name, pose, dimensions, frame_id="base_link"):
        """
        Add a box collision object to the planning scene
        
        Args:
            name (str): Unique name for the collision object
            pose (list): [x, y, z, qx, qy, qz, qw] position and orientation
            dimensions (list): [length, width, height] in meters
            frame_id (str): Reference frame (default: "base_link")
        
        Returns:
            bool: True if successfully added, False otherwise
        """
        if not self.apply_planning_scene_client.service_is_ready():
            self.get_logger().error("Planning scene service not available")
            return False
        
        # Create collision object
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = name
        
        # Create box primitive
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions
        
        # Set pose
        pose_msg = Pose()
        pose_msg.position = Point(x=pose[0], y=pose[1], z=pose[2])
        if len(pose) == 7:  # quaternion provided
            pose_msg.orientation = Quaternion(x=pose[3], y=pose[4], z=pose[5], w=pose[6])
        else:  # use identity quaternion
            pose_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        collision_object.primitives = [box]
        collision_object.primitive_poses = [pose_msg]
        collision_object.operation = CollisionObject.ADD
        
        # Create planning scene diff
        planning_scene_diff = PlanningScene()
        planning_scene_diff.is_diff = True
        planning_scene_diff.world.collision_objects = [collision_object]
        
        # Apply the planning scene
        try:
            request = ApplyPlanningScene.Request()
            request.scene = planning_scene_diff
            
            future = self.apply_planning_scene_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done() and future.result().success:
                self.get_logger().info(f"Added collision object '{name}' to planning scene")
                return True
            else:
                self.get_logger().error(f"Failed to add collision object '{name}'")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Exception adding collision object '{name}': {e}")
            return False

    def attach_tool_collision_object(self, name, dimensions, link_name="tool0", pose_offset=None):
        """
        Attach a tool collision object to the robot end-effector
        
        Args:
            name (str): Unique name for the attached tool
            dimensions (list): [length, width, height] in meters
            link_name (str): Link to attach to (default: "tool0")
            pose_offset (list): Optional [x, y, z, qx, qy, qz, qw] offset from link
        
        Returns:
            bool: True if successfully attached, False otherwise
        """
        if not self.apply_planning_scene_client.service_is_ready():
            self.get_logger().error("Planning scene service not available")
            return False
        
        # Create attached collision object
        attached_object = AttachedCollisionObject()
        attached_object.link_name = link_name
        attached_object.object.header.frame_id = link_name
        attached_object.object.header.stamp = self.get_clock().now().to_msg()
        attached_object.object.id = name
        
        # Create box primitive for tool
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions
        
        # Set pose (offset from link)
        pose_msg = Pose()
        if pose_offset and len(pose_offset) >= 3:
            pose_msg.position = Point(x=pose_offset[0], y=pose_offset[1], z=pose_offset[2])
            if len(pose_offset) == 7:
                pose_msg.orientation = Quaternion(x=pose_offset[3], y=pose_offset[4], z=pose_offset[5], w=pose_offset[6])
            else:
                pose_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        else:
            pose_msg.position = Point(x=0.0, y=0.0, z=0.0)
            pose_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
        attached_object.object.primitives = [box]
        attached_object.object.primitive_poses = [pose_msg]
        attached_object.object.operation = CollisionObject.ADD
        
        # Set touch links (links that can touch the attached object)
        attached_object.touch_links = [link_name, "wrist_3_link", "wrist_2_link"]
        
        # Create planning scene diff
        planning_scene_diff = PlanningScene()
        planning_scene_diff.is_diff = True
        planning_scene_diff.robot_state.attached_collision_objects = [attached_object]
        
        # Apply the planning scene
        try:
            request = ApplyPlanningScene.Request()
            request.scene = planning_scene_diff
            
            future = self.apply_planning_scene_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done() and future.result().success:
                self.get_logger().info(f"Attached tool '{name}' to {link_name}")
                return True
            else:
                self.get_logger().error(f"Failed to attach tool '{name}'")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Exception attaching tool '{name}': {e}")
            return False

    def remove_collision_object(self, name):
        """
        Remove a collision object from the planning scene
        
        Args:
            name (str): Name of the collision object to remove
        
        Returns:
            bool: True if successfully removed, False otherwise
        """
        if not self.apply_planning_scene_client.service_is_ready():
            self.get_logger().error("Planning scene service not available")
            return False
        
        # Create collision object for removal
        collision_object = CollisionObject()
        collision_object.id = name
        collision_object.operation = CollisionObject.REMOVE
        
        # Create planning scene diff
        planning_scene_diff = PlanningScene()
        planning_scene_diff.is_diff = True
        planning_scene_diff.world.collision_objects = [collision_object]
        
        # Apply the planning scene
        try:
            request = ApplyPlanningScene.Request()
            request.scene = planning_scene_diff
            
            future = self.apply_planning_scene_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done() and future.result().success:
                self.get_logger().info(f"Removed collision object '{name}'")
                return True
            else:
                self.get_logger().error(f"Failed to remove collision object '{name}'")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Exception removing collision object '{name}': {e}")
            return False


# Convenience functions for quick usage
def initialize_robot(node_name="ur3e_lib_node"):
    """Initialize ROS and create robot controller"""
    if not rclpy.ok():
        rclpy.init()
    return UR3eController(node_name)


def shutdown_robot():
    """Shutdown ROS"""
    if rclpy.ok():
        rclpy.shutdown()
