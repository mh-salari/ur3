#!/usr/bin/env python3
"""
Interactive UR3e Robot CLI - Joint Control Only

An interactive command-line interface for controlling the UR3e robot.
Supports only joint control (degrees) for reliability.

Usage:
    python3 ur3e_cli.py

Then follow the interactive prompts to control the robot.
"""

import rclpy
import subprocess
import readline  # Enable arrow keys and command history in input()
from ur3e_lib import UR3eController


class UR3eCLI:
    """Interactive Command Line Interface for UR3e Robot - Joint Control Only"""

    def __init__(self):
        self.robot = None
        self.running = True

    def cleanup_old_cli_nodes(self):
        """Kill any existing CLI nodes to prevent conflicts"""
        try:
            # Kill any existing ur3e_cli_node processes
            result = subprocess.run(
                ["pkill", "-f", "ur3e__cli_node"], capture_output=True, text=True
            )
            if result.returncode == 0:
                print(" Cleaned up previous CLI nodes")
        except Exception:
            pass  # Ignore cleanup errors

    def initialize_robot(self):
        """Initialize the robot connection"""
        try:
            print(" Initializing UR3e robot connection...")

            # First, cleanup any old CLI nodes
            self.cleanup_old_cli_nodes()

            if not rclpy.ok():
                rclpy.init()
            self.robot = UR3eController("ur3e__cli_node")
            print(" Robot connected successfully!")
            return True
        except Exception as e:
            print(f" Failed to connect to robot: {e}")
            return False

    def shutdown(self):
        """Shutdown the robot connection"""
        if self.robot:
            self.robot.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(" Goodbye!")

    def show_current_position(self):
        """Show current robot position"""
        if not self.robot:
            print(" Robot not connected")
            return

        try:
            current_joints = self.robot.get_current_joint_positions()
            if current_joints:
                joint_names = [
                    "base",
                    "shoulder",
                    "elbow",
                    "wrist1",
                    "wrist2",
                    "wrist3",
                ]
                print("Current joint positions:")
                for name, angle in zip(joint_names, current_joints):
                    print(f"{name}: {angle:.2f}°")
            else:
                print(" Could not get current position")
        except Exception as e:
            print(f" Error getting position: {e}")

    def move_to_home(self):
        """Move robot to home position"""
        if not self.robot:
            print(" Robot not connected")
            return

        print(" Moving to home position [0, -90, 0, -90, 0, 0]...")
        try:
            success = self.robot.go_home()
            if success:
                print(" Home position reached!")
            else:
                print(" Failed to reach home position")
        except Exception as e:
            print(f" Error moving to home: {e}")

    def move_to_joint_position(self):
        """Move robot to specified joint positions - Interactive mode"""
        if not self.robot:
            print(" Robot not connected")
            return

        print("\n" + "=" * 50)
        print("Joint Control Mode - Enter joint positions")
        print("=" * 50)
        print("Format: base, shoulder, elbow, wrist1, wrist2, wrist3")
        print("Home position:   0, -90, 0, -90, 0, 0")
        print("Valid example:   0, -60, -30, -90, 0, 0")
        print("Invalid example: 0, -90, 160, -90, 0, 0")
        print("Safe ranges: ±180° for most joints")
        print("Press Ctrl+C to return to main menu")
        print("-" * 50)

        while True:
            try:
                user_input = input("Joint angles: ").strip()

                if not user_input:
                    print(" No input provided")
                    continue

                # Parse input - handle both comma-separated and space-separated
                # Remove all spaces and split by comma, then split by spaces if no commas
                if "," in user_input:
                    joint_values = [
                        float(x.strip()) for x in user_input.split(",") if x.strip()
                    ]
                else:
                    joint_values = [float(x) for x in user_input.split()]

                if len(joint_values) != 6:
                    print(" Please provide exactly 6 joint angles")
                    continue

                # Validate ranges (basic check)
                valid = True
                for i, angle in enumerate(joint_values):
                    if abs(angle) > 360:
                        print(f"  Warning: Joint {i + 1} angle {angle}° is very large")
                        confirm = input("Continue anyway? (y/n): ").strip().lower()
                        if confirm not in ["y", "yes"]:
                            print(" Movement cancelled")
                            valid = False
                            break

                if not valid:
                    continue

                print(
                    f" Moving to joint positions: {[round(j, 2) for j in joint_values]}°"
                )

                success = self.robot.move_to_joint_positions(joint_values)
                if success:
                    print(" Movement completed!")
                else:
                    print(" Movement failed")

            except ValueError:
                print(" Invalid input. Please enter 6 numbers separated by spaces")
            except KeyboardInterrupt:
                print()  # Just a newline for clean formatting
                break
            except Exception as e:
                print(f" Error during movement: {e}")

    def show_menu(self):
        """Show the main menu"""
        print("\n" + "=" * 50)
        print(" UR3e Robot Interactive Controller")
        print("=" * 50)
        print("1. Show current joint positions")
        print("2. Go to home position")
        print("3. Move to joint positions (degrees)")
        print("4. Quit")
        print("-" * 50)
        print("Note: Only joint control available for safety")

    def run(self):
        """Run the interactive CLI"""
        print(" Starting UR3e  CLI (Joint Control Only)")

        # Initialize robot
        if not self.initialize_robot():
            return

        print("\n Ready to control the robot!")

        # Main loop
        while self.running:
            try:
                self.show_menu()
                choice = input("Choose option (1-4): ").strip()

                if choice == "1":
                    self.show_current_position()
                    if self.running:
                        input("\nPress Enter to continue...")

                elif choice == "2":
                    self.move_to_home()
                    if self.running:
                        input("\nPress Enter to continue...")

                elif choice == "3":
                    self.move_to_joint_position()
                    # No pause needed - joint mode handles its own loop

                elif choice == "4" or choice.lower() == "quit":
                    self.running = False

                else:
                    print(" Invalid choice. Please enter 1-4")
                    if self.running:
                        input("\nPress Enter to continue...")

            except KeyboardInterrupt:
                print("\n\n Interrupted by user")
                self.running = False

            except Exception as e:
                print(f" Unexpected error: {e}")
                print("Continuing...")

        # Shutdown
        self.shutdown()


def main():
    """Main entry point"""
    cli = UR3eCLI()
    cli.run()


if __name__ == "__main__":
    main()
