# UR3e Robot Control System

*Created by Mohammad Hossein Salari for Industrial Robotics and Automated Systems*  
*UEF Summer School Courses 2025*

![It's not much, but it's honest work](assets/But_It's_Honest_Work.jpg)

## Overview

A complete ROS 2 + MoveIt 2 control system for Universal Robots UR3e with Python CLI interface. Supports safe joint control with collision avoidance and headless operation.

## Quick Start

1. **Installation**: Follow `01_Installation_Guide.md` 
2. **Usage**: See `02_Running_Guide.md` for step-by-step execution

## Files Structure

- **`01_Installation_Guide.md`** - Complete installation instructions
- **`02_Running_Guide.md`** - Step-by-step usage guide  
- **`setup_ur3_functions.sh`** - Bash function configuration
- **`code/`** - Python control interface
  - `ur3e_cli.py` - Interactive command-line interface
  - `ur3e_lib.py` - Robot control library
  - `ur3_types.py` - Joint position type definitions
  - `cli.py` - Updated CLI with type support
  - `demo.py` - Demo routine using typed joints
- **`assets/`** - Images and resources

## Features

- ✅ Control the joints position UR3e with Pyhton!
- ✅ Safe joint control with MoveIt planning
- ✅ Collision avoidance and safety checks


## Commands

```bash
ur3 run driver    # Start UR3e driver (headless mode)
ur3 run moveit    # Start MoveIt planning server
uv run cli.py     # Launch interactive control CLI
```

---
*Industrial Robotics Project - Summer 2025*
