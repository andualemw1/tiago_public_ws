# TIAGo ROS 2 Humble Simulation Workspace

This repository contains a ROS 2 Humble workspace for simulating and controlling the **TIAGo mobile manipulator** using **Gazebo Classic**.  
The project focuses on navigation, arm manipulation, and task execution within a simulated environment.

Due to current compatibility constraints between TIAGo, Gazebo Fortress, and MoveIt 2, arm manipulation is performed by **directly commanding joint state values** rather than relying on full inverse kinematics planning.

---

## System Configuration

- **Operating System:** Ubuntu 22.04
- **ROS Version:** ROS 2 Humble
- **Simulator:** Gazebo Classic (Gazebo 11)
- **Robot Platform:** PAL Robotics TIAGo
- **Manipulation Approach:** Direct joint-state control
- **Navigation:** Velocity-based control (`/cmd_vel`)

> ⚠️ TIAGo official simulation support for ROS 2 Humble is currently limited to Gazebo Classic. Gazebo Fortress is not supported due to plugin and controller incompatibilities.

---

## Workspace Structure

```text
tiago_public_ws/
├── src/                     # TIAGo and PAL Robotics packages (submodules)
├── scripts/                 # Custom Python control scripts
│   ├── go_to_xy.py
│   ├── lift_arm.py
│   ├── arm_to_xyz.py
│   ├── tiago_launch.py
│   └── tiago_pp.py
├── configs/                 # Jupyter notebooks and instructions
├── .gitignore
└── README.md


## Simulation Demonstrations

## Simulation Demonstrations

### Demo 1: TIAGo Navigation
![TIAGo navigation](https://raw.githubusercontent.com/andualemw1/tiago_public_ws/main/docs/figures/exercise_101.png)

### Demo 2: Pick-and-Place Task
![TIAGo pick-and-place](https://raw.githubusercontent.com/andualemw1/tiago_public_ws/main/docs/figures/exercise_103.png)



