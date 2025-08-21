#!/usr/bin/env python3
"""
UR3e Simple Face Tracking Demo
Uses webcam to detect faces and moves robot to follow face position.
Uses direct joint trajectory control without MoveIt.
"""

import math
import threading
import time

import cv2
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

# FollowJointTrajectoryGoal is in control_msgs.action, not .msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ur3_types import JointPositions as Jpos


class SimpleUR3eController(Node):
    """Simplified UR3e controller for face tracking demo"""

    def __init__(self, node_name="simple_face_tracking"):
        super().__init__(node_name)

        # Create action client for trajectory control
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        )

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(JointState, "joint_states", self._joint_state_callback, 10)

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

        # Wait for connections
        self.get_logger().info("Connecting to trajectory controller...")
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Failed to connect to trajectory controller")
            raise Exception("Trajectory controller not available")

        # Wait for joint state
        while not self.joint_state_received:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Simple controller ready!")

    def _joint_state_callback(self, msg):
        """Store current joint state"""
        self.current_joint_state = msg
        self.joint_state_received = True

    def move_to_joint_positions(self, joint_positions, duration=2.0):
        """Move robot to specified joint positions (in degrees)"""
        if not self.joint_state_received:
            self.get_logger().warn("No joint state received yet")
            return False

        # Convert degrees to radians
        target_positions = [math.radians(pos) for pos in joint_positions.to_list()]

        # Create trajectory goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.ur_joint_names

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        goal.trajectory.points = [point]

        # Send goal
        future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if future.result() is not None:
            return future.result().accepted

        return False


class FaceTrackingDemo:
    def __init__(self):
        self.robot = None
        self.face_cascade = None
        self.cap = None
        self.running = False
        self.face_center = None
        self.frame_width = 640
        self.frame_height = 480

        # Robot movement parameters
        self.base_center = 0  # Center position for base joint
        self.shoulder_center = -90  # Center position for shoulder
        self.elbow_center = 0  # Center position for elbow
        self.max_base_angle = 60  # Maximum base rotation (degrees)
        self.max_shoulder_angle = 30  # Maximum shoulder movement (degrees)
        self.max_elbow_angle = 40  # Maximum elbow movement (degrees)

        # Safe robot positions
        self.home_position = Jpos([0, -90, 0, -90, 0, 0])
        self.current_position = self.home_position.copy()

    def initialize_robot(self):
        """Initialize the robot controller"""
        rclpy.init()
        self.robot = SimpleUR3eController("face_tracking_demo")

        # Move to home position
        print("Moving robot to home position...")
        self.robot.move_to_joint_positions(self.home_position)
        self.current_position = self.home_position.copy()
        print("Robot initialized and ready for face tracking")

    def initialize_camera(self):
        """Initialize webcam and face detection"""
        try:
            # Load OpenCV's pre-trained face detector
            self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

            # Initialize webcam
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                raise Exception("Could not open webcam")

            # Set camera resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

            print("Camera initialized successfully")
            return True

        except Exception as e:
            print(f"Error initializing camera: {e}")
            return False

    def detect_faces(self, frame):
        """Detect faces in the frame and return the center of the largest face"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)

        if len(faces) > 0:
            # Find the largest face
            largest_face = max(faces, key=lambda face: face[2] * face[3])
            x, y, w, h = largest_face

            # Calculate face center
            face_center_x = x + w // 2
            face_center_y = y + h // 2

            # Draw rectangle around face
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.circle(frame, (face_center_x, face_center_y), 5, (0, 255, 0), -1)

            return (face_center_x, face_center_y)

        return None

    def calculate_robot_position(self, face_center):
        """Calculate robot joint angles based on face position"""
        if face_center is None:
            return self.current_position

        face_x, face_y = face_center

        # Normalize face coordinates to [-1, 1]
        normalized_x = (face_x - self.frame_width / 2) / (self.frame_width / 2)
        normalized_y = (face_y - self.frame_height / 2) / (self.frame_height / 2)

        # Calculate new joint angles
        new_position = self.home_position.copy()  # Start from home position

        # Base joint follows horizontal face movement (inverted for natural tracking)
        base_angle = self.base_center - (normalized_x * self.max_base_angle)
        new_position.base = max(-90, min(90, base_angle))

        # Shoulder joint follows vertical face movement (up/down)
        # When face is up (negative y), shoulder should lift (less negative)
        shoulder_angle = self.shoulder_center - (normalized_y * self.max_shoulder_angle)
        new_position.shoulder = max(-140, min(-40, shoulder_angle))

        # Elbow joint also contributes to vertical movement for better range
        # When face is down, elbow bends more positive
        elbow_angle = self.elbow_center + (normalized_y * self.max_elbow_angle)
        new_position.elbow = max(-60, min(60, elbow_angle))

        return new_position

    def robot_control_loop(self):
        """Main robot control loop running in separate thread"""
        while self.running:
            if self.face_center is not None:
                target_position = self.calculate_robot_position(self.face_center)

                # Only move if position changed significantly
                if self.position_changed(target_position, self.current_position):
                    try:
                        if self.robot.move_to_joint_positions(target_position, duration=0.5):
                            self.current_position = target_position
                    except Exception as e:
                        print(f"Robot movement error: {e}")

            time.sleep(0.2)  # 5Hz control loop

    def position_changed(self, new_pos, old_pos, threshold=2.0):
        """Check if position changed significantly"""
        return (
            abs(new_pos.base - old_pos.base) > threshold
            or abs(new_pos.shoulder - old_pos.shoulder) > threshold
            or abs(new_pos.elbow - old_pos.elbow) > threshold
        )

    def run(self):
        """Main demo loop"""
        try:
            # Initialize robot
            self.initialize_robot()

            # Initialize camera
            if not self.initialize_camera():
                return

            self.running = True

            # Start robot control thread
            robot_thread = threading.Thread(target=self.robot_control_loop)
            robot_thread.daemon = True
            robot_thread.start()

            print("Face tracking demo started!")
            print("Press 'q' to quit, 'h' to return robot to home position")

            while self.running:
                ret, frame = self.cap.read()
                if not ret:
                    print("Error reading from camera")
                    break

                # Flip frame horizontally for mirror effect
                frame = cv2.flip(frame, 1)

                # Detect faces
                face_center = self.detect_faces(frame)
                self.face_center = face_center

                # Draw crosshair at frame center
                cv2.line(
                    frame,
                    (self.frame_width // 2 - 20, self.frame_height // 2),
                    (self.frame_width // 2 + 20, self.frame_height // 2),
                    (0, 255, 255),
                    2,
                )
                cv2.line(
                    frame,
                    (self.frame_width // 2, self.frame_height // 2 - 20),
                    (self.frame_width // 2, self.frame_height // 2 + 20),
                    (0, 255, 255),
                    2,
                )

                # Display status and coordinates
                if face_center:
                    status_text = f"Face: ({face_center[0]}, {face_center[1]})"
                    # Calculate normalized coordinates for display
                    norm_x = (face_center[0] - self.frame_width / 2) / (self.frame_width / 2)
                    norm_y = (face_center[1] - self.frame_height / 2) / (self.frame_height / 2)
                    coord_text = f"Norm: ({norm_x:.2f}, {norm_y:.2f})"
                    cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame, coord_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    status_text = "No face detected"
                    cv2.putText(frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # Show frame
                cv2.imshow("Face Tracking Demo", frame)

                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("h"):
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
        """Clean up resources"""
        print("Cleaning up...")
        self.running = False

        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()

        if self.robot:
            try:
                # Return to home position
                print("Returning robot to home position...")
                self.robot.move_to_joint_positions(self.home_position)
                self.robot.destroy_node()
            except:
                pass

        rclpy.shutdown()
        print("Demo finished!")


def main():
    demo = FaceTrackingDemo()
    demo.run()


if __name__ == "__main__":
    main()
