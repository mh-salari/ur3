#!/usr/bin/env python3
"""
UR3e Simple Color Tracking Demo
Tracks the color red using webcam and moves robot to follow the red object position.
"""
 
import rclpy
import cv2
import numpy as np
import time
import threading
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur3_types import JointPositions as Jpos
 
class SimpleUR3eController(Node):
    def __init__(self, node_name="simple_color_tracking"):
        super().__init__(node_name)
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        )
        self.joint_state_sub = self.create_subscription(
            JointState, "joint_states", self._joint_state_callback, 10
        )
        self.current_joint_state = None
        self.joint_state_received = False
        self.ur_joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.get_logger().info("Connecting to trajectory controller...")
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Failed to connect to trajectory controller")
            raise Exception("Trajectory controller not available")
        while not self.joint_state_received:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Simple controller ready!")
 
    def _joint_state_callback(self, msg):
        self.current_joint_state = msg
        self.joint_state_received = True
 
    def move_to_joint_positions(self, joint_positions, duration=2.0):
        if not self.joint_state_received:
            self.get_logger().warn("No joint state received yet")
            return False
        target_positions = [math.radians(pos) for pos in joint_positions.to_list()]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.ur_joint_names
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        goal.trajectory.points = [point]
        future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.result() is not None:
            return future.result().accepted
        return False
 
class ColorTrackingDemo:
    def __init__(self):
        self.robot = None
        self.cap = None
        self.running = False
        self.object_center = None
        self.frame_width = 640
        self.frame_height = 480
        self.base_center = 0
        self.shoulder_center = -90
        self.elbow_center = 0
        self.max_base_angle = 60
        self.max_shoulder_angle = 30
        self.max_elbow_angle = 40
        self.home_position = Jpos([0, -90, 0, -90, 0, 0])
        self.current_position = self.home_position.copy()
 
    def initialize_robot(self):
        rclpy.init()
        self.robot = SimpleUR3eController("color_tracking_demo")
        print("Moving robot to home position...")
        self.robot.move_to_joint_positions(self.home_position)
        self.current_position = self.home_position.copy()
        print("Robot initialized and ready for color tracking")
 
    def initialize_camera(self):
        try:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                raise Exception("Could not open webcam")
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            print("Camera initialized successfully")
            return True
        except Exception as e:
            print(f"Error initializing camera: {e}")
            return False
 
    def detect_red_object(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 500:
                (x, y, w, h) = cv2.boundingRect(largest)
                center_x = x + w // 2
                center_y = y + h // 2
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
                return (center_x, center_y)
        return None
 
    def calculate_robot_position(self, object_center):
        if object_center is None:
            return self.current_position
        obj_x, obj_y = object_center
        normalized_x = (obj_x - self.frame_width / 2) / (self.frame_width / 2)
        normalized_y = (obj_y - self.frame_height / 2) / (self.frame_height / 2)
        new_position = self.home_position.copy()
        base_angle = self.base_center - (normalized_x * self.max_base_angle)
        new_position.base = max(-90, min(90, base_angle))
        shoulder_angle = self.shoulder_center - (normalized_y * self.max_shoulder_angle)
        new_position.shoulder = max(-140, min(-40, shoulder_angle))
        elbow_angle = self.elbow_center + (normalized_y * self.max_elbow_angle)
        new_position.elbow = max(-60, min(60, elbow_angle))
        return new_position
 
    def robot_control_loop(self):
        while self.running:
            if self.object_center is not None:
                target_position = self.calculate_robot_position(self.object_center)
                if self.position_changed(target_position, self.current_position):
                    try:
                        if self.robot.move_to_joint_positions(target_position, duration=0.5):
                            self.current_position = target_position
                    except Exception as e:
                        print(f"Robot movement error: {e}")
            time.sleep(0.2)
 
    def position_changed(self, new_pos, old_pos, threshold=2.0):
        return (abs(new_pos.base - old_pos.base) > threshold or 
                abs(new_pos.shoulder - old_pos.shoulder) > threshold or
                abs(new_pos.elbow - old_pos.elbow) > threshold)
 
    def run(self):
        try:
            self.initialize_robot()
            if not self.initialize_camera():
                return
            self.running = True
            robot_thread = threading.Thread(target=self.robot_control_loop)
            robot_thread.daemon = True
            robot_thread.start()
            print("Color tracking demo started!")
            print("Press 'q' to quit, 'h' to return robot to home position")
            while self.running:
                ret, frame = self.cap.read()
                if not ret:
                    print("Error reading from camera")
                    break
                frame = cv2.flip(frame, 1)
                object_center = self.detect_red_object(frame)
                self.object_center = object_center
                cv2.line(frame, (self.frame_width//2 - 20, self.frame_height//2), 
                        (self.frame_width//2 + 20, self.frame_height//2), (0, 255, 255), 2)
                cv2.line(frame, (self.frame_width//2, self.frame_height//2 - 20), 
                        (self.frame_width//2, self.frame_height//2 + 20), (0, 255, 255), 2)
                if object_center:
                    status_text = f"Red: ({object_center[0]}, {object_center[1]})"
                    norm_x = (object_center[0] - self.frame_width / 2) / (self.frame_width / 2)
                    norm_y = (object_center[1] - self.frame_height / 2) / (self.frame_height / 2)
                    coord_text = f"Norm: ({norm_x:.2f}, {norm_y:.2f})"
                    cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame, coord_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    status_text = "No red object detected"
                    cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow('Color Tracking Demo', frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('h'):
                    print("Returning robot to home position...")
                    self.robot.move_to_joint_positions(self.home_position)
                    self.current_position = self.home_position.copy()
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        except Exception as e:
            print(f"Demo error: {e}")
        finally:
            self.cleanup()
 
    def cleanup(self):
        print("Cleaning up...")
        self.running = False
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        if self.robot:
            try:
                print("Returning robot to home position...")
                self.robot.move_to_joint_positions(self.home_position)
                self.robot.destroy_node()
            except:
                pass
        rclpy.shutdown()
        print("Demo finished!")
 
def main():
    demo = ColorTrackingDemo()
    demo.run()
 
if __name__ == "__main__":
    main()
 