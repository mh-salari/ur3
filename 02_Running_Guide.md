# UR3 + MoveIt 2 Running Guide

> **Step-by-step execution guide** for ROS 2 Jazzy + MoveIt 2 + moveit_py Python API

---

## Prerequisites Checklist

Before starting, ensure you have completed:
- ✅ **Installation Guide** (`01_Installation_Guide.md`) - All software installed
- ✅ **Robot setup** - External Control URCap installed and program created
- ✅ **Network setup** - Robot and PC connected via Ethernet
- ✅ **Bash functions** - `ur3` commands configured in Step 10 of Installation Guide
- ✅ **Python CLI** - `code/` directory with dependencies installed using `uv sync`

---

## Quick Session Overview

1. **Terminal A:** Start UR ROS 2 driver
2. **Terminal B:** Start MoveIt 2 move_group 
3. **Terminal C:** Run verification commands
4. **Terminal D:** Execute Python CLI demo

---

## Step-by-Step Execution


### Step 1: Start UR ROS 2 Driver (Terminal A)

Launch UR driver with calibration and RViz:
```bash
ur3 run driver launch_rviz:=true
```

**Note:** This automatically includes your calibration file for improved accuracy.

**Expected output:**
```
[INFO] [ur_control_node]: Waiting for robot program to be running...
[INFO] [ur_control_node]: Robot connected to control node!
```

**❗ Important:** After seeing the connection message, proceed to the teach pendant.

### Step 2: Activate Robot Program (Robot Teach Pendant)

**Note:** The **teach pendant** is the tablet-ish screen tethered to the UR3 that runs PolyScope (the robot's onboard UI).

1. On the UR3 teach pendant, navigate to your saved `external_control.urp` program
2. Select the program containing the **External Control** node  
3. Press the **▶ Play** button
4. The program should start and show "External Control" active

**In Terminal A, you should now see:**
```
[INFO] [ur_control_node]: Robot program is running
[INFO] [ur_control_node]: Controllers active: [scaled_joint_trajectory_controller, io_and_status_controller]
```

### Step 3: Start MoveIt 2 (Terminal B)

Launch MoveIt 2 with UR3 configuration and RViz:
```bash
ur3 run moveit launch_rviz:=true
```

**Expected output:**
```
[INFO] [move_group]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[INFO] [move_group]: MoveGroup context initialization complete
```

**RViz should open** showing the UR3 robot model and MoveIt motion planning interface.

### Step 4: Verify System Status (Terminal C)

```bash
# Check ROS 2 topics
echo "=== ROS 2 Topics ==="
ros2 topic list | grep -E "(joint_states|trajectory|robot_description)"

# Check controllers
echo "=== Controllers Status ==="
ros2 control list_controllers

# Test robot connectivity
echo "=== Robot Connectivity ==="
ping -c 3 192.168.1.102

# Check MoveIt services
echo "=== MoveIt Services ==="
ros2 service list | grep move_group
```

**Expected controller output:**
```
scaled_joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active
io_and_status_controller[ur_controllers/GPIOController] active
```

### Step 5: Run Interactive Python CLI (Terminal D)

Navigate to the Python CLI directory:
```bash
cd ~/ur3/code/
```

Start the interactive CLI:
```bash
uv run cli.py
```

**Expected output:**
```
 Starting UR3e  CLI (Joint Control Only)
 Initializing UR3e robot connection...
 Robot connected successfully!

 Ready to control the robot!

==================================================
 UR3e Robot Interactive Controller
==================================================
1. Show current joint positions
2. Go to home position
3. Move to joint positions (degrees)
4. Quit
--------------------------------------------------
Note: Only joint control available for safety
Choose option (1-4):
```

---

## Interactive CLI Usage

### Safe test joint positions for UR3:
In the interactive CLI, choose Option 3: Move to joint positions (degrees)

Home position:
```
0 -90 0 -90 0 0
```

Safe raised position:
```
0 -60 -30 -90 0 0
```

Left side position:
```
90 -90 0 -90 0 0
```

Right side position:
```
-90 -90 0 -90 0 0
```

Slightly forward:
```
0 -45 -45 -90 0 0
```

**Format:** base shoulder elbow wrist1 wrist2 wrist3 (all in degrees)
**Safe ranges:** Generally ±180° for most joints

### Debugging commands:
```bash
ros2 topic echo /joint_states --once
```

```bash
ros2 topic echo /scaled_joint_trajectory_controller/joint_trajectory --once
```

```bash
ros2 run tf2_tools view_frames
```

---

## Troubleshooting

### Problem: "Robot program is not running"
**Solution:** 
1. Check that External Control program (`external_control.urp`) is loaded on teach pendant
2. Press ▶ Play button on teach pendant
3. Verify network connectivity: `ping 192.168.1.102`
4. If External Control is missing from URCaps, install it following the Installation Guide

### Problem: "Planning failed" errors
**Possible causes:**
- Pose is outside robot workspace → Try poses closer to robot base
- Collision detected → Check for obstacles in RViz
- Orientation unreachable → Simplify orientation or try different yaw values

**Debug joint positions:**
Try very safe positions near home in the interactive CLI:
```
0 -75 -15 -90 0 0
```

### Problem: "moveit_py import error"
**Solution:**
```bash
ros2 pkg list | grep moveit-py
```

```bash
sudo apt install ros-jazzy-moveit-py
```

```bash
python3 -c "from moveit.planning import MoveItPy; print('OK')"
```

### Problem: Controllers not active
**Solution:**
```bash
ros2 control list_controllers
```

If inactive, restart UR driver (Terminal A) and make sure External Control is running on teach pendant.

### Problem: RViz shows no robot model
**Solution:**
1. In RViz, check that `robot_description` topic is available
2. In Displays panel, ensure RobotModel display is enabled
3. Check that `ur_moveit.launch.py` started without errors

---

## Safe Shutdown Procedure

1. **Stop Python CLI** (Ctrl+C in Terminal D)
2. **Stop External Control** on robot teach pendant (Stop button)
3. **Stop MoveIt** (Ctrl+C in Terminal B)  
4. **Stop UR Driver** (Ctrl+C in Terminal A)
5. **Close RViz windows**

---

## Session Script (Quick Reference)

For experienced users, here's the condensed startup sequence:

Terminal A: UR Driver (with calibration)
```bash
ur3 run driver launch_rviz:=true
```

Press Play on teach pendant, then Terminal B: MoveIt
```bash
ur3 run moveit launch_rviz:=true
```

Terminal C: Interactive CLI
```bash
cd ~/ur3/code/
uv run cli.py
```

Choose option 2 to go home, then option 3 to control joints.

---

## Performance Notes

- **Planning time:** Typically 1-3 seconds for simple poses
- **Execution time:** Depends on distance and velocity scaling
- **Default speeds:** Conservative (20% velocity/acceleration) for safety
- **Collision checking:** Always enabled - provides safe motion planning

---

## Next Steps

- Modify poses and parameters to explore robot workspace
- Add custom waypoints or constraints to the Python script
- Integrate with camera systems or sensors as needed
- Consider creating launch files for automated startup