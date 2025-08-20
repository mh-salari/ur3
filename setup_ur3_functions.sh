#!/bin/bash

# UR3 Setup Script - Configures shell environment for UR3 control
# Run this script once after installation to set up ROS 2 and ur3 commands

echo "Setting up UR3 shell environment..."

# Check if ROS 2 is already sourced
if grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "ROS 2 Jazzy already sourced in ~/.bashrc"
else
    echo "Adding ROS 2 Jazzy to ~/.bashrc..."
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

# Check if ur3 function already exists
if grep -q "ur3()" ~/.bashrc; then
    echo "UR3 function already exists in ~/.bashrc"
    echo "Skipping UR3 function installation to avoid duplicates"
else
    echo "Adding UR3 functions to ~/.bashrc..."
    # Add the ur3 function to .bashrc
    cat >> ~/.bashrc << 'EOF'

# UR3 Robot Control Functions
ur3() {
    case "$1" in
        "run")
            case "$2" in
                "driver")
                    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.1.102 kinematics_params_file:="${HOME}/ur3e_calibration.yaml" launch_rviz:=false "${@:3}"
                    ;;
                "moveit")
                    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=false "${@:3}"
                    ;;
                *)
                    echo "Usage: ur3 run [driver|moveit] [options]"
                    ;;
            esac
            ;;
        *)
            echo "Usage: ur3 run [driver|moveit] [options]"
            ;;
    esac
}
EOF
fi

echo ""
echo "Setup complete! Run 'source ~/.bashrc' or restart your terminal to use the new commands."
echo ""
echo "Available commands:"
echo "  ur3 run driver                    - Start UR3 driver with calibration"
echo "  ur3 run driver launch_rviz:=true  - Start UR3 driver with RViz"
echo "  ur3 run moveit                    - Start MoveIt"
echo "  ur3 run moveit launch_rviz:=true  - Start MoveIt with RViz"