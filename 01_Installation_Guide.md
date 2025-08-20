# UR3 + ROS 2 + MoveIt 2 — Installation Guide

> **Compatible Versions:** ROS 2 Jazzy LTS + MoveIt 2.14.0 + Ubuntu 24.04 LTS + Universal Robots ROS 2 Driver

---

## Prerequisites

- **Operating System:** Ubuntu 24.04 LTS (recommended)
- **Hardware:** UR3 robot with e-Series or CB3 control box
- **Network:** Direct Ethernet connection between PC and robot controller

---

## Step 1: Install ROS 2 Jazzy (Ubuntu 24.04)

### Set locale
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### Setup Sources
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### Add ROS 2 repository
```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

### Install ROS 2 Jazzy
```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop
```

### Setup environment
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 2: Install MoveIt 2 for Jazzy

```bash
# Install MoveIt 2 with Python bindings
sudo apt install ros-jazzy-moveit
sudo apt install ros-jazzy-moveit-py

# Install additional MoveIt packages
sudo apt install ros-jazzy-moveit-configs-utils
sudo apt install ros-jazzy-moveit-planners-ompl
sudo apt install ros-jazzy-moveit-servo
```

---

## Step 3: Install Universal Robots ROS 2 Driver

```bash
# Install UR driver and ros2_control stack
sudo apt install ros-jazzy-ur
sudo apt install ros-jazzy-ros2-control 
sudo apt install ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-ur-moveit-config

# Install additional control packages
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-xacro
```

---

## Step 4: Install Python Dependencies

```bash
# Install uv for Python package management
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc
```

## Step 5: Download and Setup Python CLI

```bash
# Download the UR3 CLI Python code to your working directory
# Place the ur3_cli/ directory in your project folder
# Example: /home/user/UR3_UEF/ur3_cli/

# Navigate to the Python CLI directory
cd ur3_cli/

# Install dependencies using uv
uv sync

# Make the script executable
chmod +x goto_pose.py
```

---

## Step 6: Verify Installation

### Test ROS 2 installation
```bash
source /opt/ros/jazzy/setup.bash
ros2 doctor
```
Expected output should show successful checks and `ROS_DISTRO=jazzy`

Alternative quick test:
```bash
printenv | grep ROS
```
Should show: `ROS_VERSION=2`, `ROS_PYTHON_VERSION=3`, `ROS_DISTRO=jazzy`

### Test MoveIt installation
```bash
ros2 pkg list | grep moveit
```
Should show multiple moveit packages including `moveit_py`

### Test UR driver installation
```bash
ros2 pkg list | grep ur
```
Should show ur-related packages including `ur_robot_driver` and `ur_moveit_config`

---

## Step 7: Robot Setup

### External Control URCap Installation

**Important:** The robot **teach pendant** (the tablet-ish screen tethered to the UR3) must have the **External Control** URCap installed to work with ROS 2.

#### Check if External Control is already installed:
1. On the teach pendant → **Program → New Program → Add → URCaps**
2. Look for **External Control** in the list
3. If it's there, skip to "Create External Control Program" below

**📺 Video Tutorial:** [Universal Robots External Control URCap Installation](https://www.youtube.com/watch?v=w1-l162NiME)

#### If External Control is NOT installed:
1. **Download URCap:** Get the latest External Control URCap (`.urcap` file) from [Universal Robots GitHub](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap)
2. **Copy to USB:** Put the `.urcap` file on a USB stick
3. **Enable System tab access (if grayed out):**
   - Put the robot in **Manual mode**
   - Default password is **12345678**
   - Exit remote control mode if active
4. **Install on pendant:** 
   - Pendant → **Settings → System → URCaps → +** 
   - Select the `.urcap` file from USB → **Install**
   - Reboot PolyScope when prompted

#### Create External Control Program:
1. Pendant → **Program → New Program → Add → URCaps → External Control**
2. Set **Remote host/IP** to your PC's IP address (same subnet as robot)
3. Leave port at **50002** (default)
4. **Save** as `external_control.urp`

**To get robot IP:** Check the robot pendant → About page when robot is booted.

---

## Step 8: Network Configuration

1. **Connect robot and PC via Ethernet cable** (direct connection recommended)

2. **Configure network settings:**
   ```bash
   # Example network configuration:
   # Robot IP: 192.168.1.102
   # PC IP: 192.168.1.50
   # Subnet: 255.255.255.0
   ```

3. **Test connectivity:**
   ```bash
   ping 192.168.1.102
   ```

---

## Step 9: Download Robot Calibration (Recommended)

Each UR robot has unique factory calibration data that improves positional accuracy. This step downloads the calibration directly from your robot.

1. **Ensure robot is powered on** (can be idle - no program needs to be running)
2. **Download the factory calibration:**
   ```bash
   ros2 launch ur_calibration calibration_correction.launch.py \
       robot_ip:=192.168.1.102 \
       target_filename:="${HOME}/ur3e_calibration.yaml"
   ```

3. **Calibration file will be saved** to `~/ur3e_calibration.yaml`

**Note:** Without this calibration, end effector positions may be off by several centimeters. Keep this file for future use.

---

## Step 10: Create Convenient Bash Aliases

Create shortcuts for the long UR3 commands to make daily use easier.

### Create a ur3 function in your shell configuration:

```bash
# Add this function to ~/.bashrc (or ~/.zshrc if using zsh)
cat >> ~/.bashrc << 'EOF'
ur3() {
    case "$1" in
        "run")
            case "$2" in
                "driver")
                    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.1.102 kinematics_params_file:="${HOME}/ur3e_calibration.yaml" "${@:3}"
                    ;;
                "moveit")
                    ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 "${@:3}"
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

# Reload your shell configuration
source ~/.bashrc
```

### Usage after installation:

```bash
# Start UR3 driver with calibration (Terminal A)
ur3 run driver

# Start UR3 driver with RViz
ur3 run driver launch_rviz:=true

# Start MoveIt (Terminal B) 
ur3 run moveit

# Start MoveIt with RViz
ur3 run moveit launch_rviz:=true
```

**Note:** The `ur3 run driver` command automatically includes your calibration file for improved accuracy. Add `launch_rviz:=true` when you want to open RViz.

---

## Version Summary

- **Ubuntu:** 24.04 LTS
- **ROS 2:** Jazzy Jalisco LTS
- **MoveIt:** 2.14.0 with moveit_py Python API
- **UR Driver:** Latest Jazzy-compatible version
- **Python API:** moveit_py (replaces deprecated moveit_commander)

---

## Next Steps

After successful installation:
1. Navigate to the **ur3_cli/** directory 
2. Follow **02_Running_Guide.md** for step-by-step execution instructions

---

## Troubleshooting

- **ROS 2 not found:** Ensure you've sourced the setup script: `source /opt/ros/jazzy/setup.bash`
- **moveit_py import errors:** Verify installation: `ros2 pkg list | grep moveit-py`
- **Network issues:** Use `ip route` and `ping` to verify connectivity
- **URCap issues:** Check robot software version compatibility with External Control URCap

For additional help, refer to:
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [MoveIt 2 Documentation](https://moveit.picknik.ai/)
- [Universal Robots ROS 2 Documentation](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/)