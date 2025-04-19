# Crab Steering & Drive Packages

This repository contains two ROS 2/​Ignition Gazebo packages:

1. **crab_steering_plugin**  
   A custom Ignition Gazebo Fortress system plugin that implements holonomic “crab” steering on a multi‑wheeled robot via `/cmd_vel`.

2. **drive**  
   A “drive” stack that launches your robot model in Gazebo, provides a simple `cmd_vel` subscriber, and includes world/​model assets for testing.

---

## Table of Contents

- [Features](#features)  
- [Prerequisites](#prerequisites)  
- [Workspace Layout](#workspace-layout)  
- [Building](#building)  
- [Package: crab_steering_plugin](#package-crab_steering_plugin)  
  - [What It Does](#what-it-does)  
  - [Usage](#usage)  
  - [Plugin Parameters (SDF)](#plugin-parameters-sdf)  
- [Package: drive](#package-drive)  
  - [What It Does](#what-it-does-1)  
  - [Usage](#usage-1)  
  - [Launching the Robot](#launching-the-robot)  
  - [Testing with `cmd_vel`](#testing-with-cmd_vel)  
- [PID Tuning](#pid-tuning)  
- [License](#license)

---

## Features

- **Holonomic Crab Steering** — steer all wheels by a target angle, drive wheels at requested speed.  
- **PID‑based Steering** — internal PID loop to command steering joint velocities to reach a desired angle.  
- **ROS 2 Integration** — subscribes to `/cmd_vel` via `rclcpp`, no external controllers required.  
- **Ignition Gazebo 6** compatibility.  
- **Drive Package** with launch files, URDF/SDF models, worlds, and a simple C++ subscriber example.

---

## Prerequisites

- Ubuntu 22.04 (or compatible)  
- ROS 2 Humble Hawksbill  
- Ignition Gazebo Fortress (gazebo6)  
- `colcon`, `ament_cmake`, `rclcpp`, `geometry_msgs`, `ignition-gazebo6`

---

## Workspace Layout

```text
nav2_ws/
├── crab_steering_plugin/      # Custom plugin package
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/
│   │   └── CrabSteeringPlugin.hpp
│   └── src/
│       └── CrabSteeringPlugin.cpp
└── drive/                     # Robot model & launch package
    ├── CMakeLists.txt
    ├── package.xml
    ├── launch/
    │   ├── gazebo.launch.py
    │   └── display.launch.py
    ├── models/
    │   └── drive/
    │       ├── urdf/drive.urdf
    │       └── meshes/…
    ├── worlds/maze.sdf
    └── scripts/
        ├── drive_node.py
        └── subscriber.cpp

Clone this repo into your workspace’s src/ directory:

mkdir -p ~/nav2_ws
cd ~/nav2_ws
git clone https://github.com/KushKuk/nav2_ws.git

Install system dependencies:

sudo apt update
sudo apt install ros-humble-ignition-gazebo6 ros-humble-rclcpp ros-humble-geometry-msgs

Build & source:

cd ~/nav2_ws
colcon build --symlink-install
source install/setup.bash

Package: crab_steering_plugin
What It Does

    Implements a System Plugin for Ignition Gazebo 6 (ign gazebo).

    Finds all joints with names containing "steer" and "wheel".

    Subscribes to /cmd_vel (ROS 2) messages.

    Computes a desired steering angle (atan2(linearY, |linearX|)).

    Runs a simple PID loop per steering joint to command joint‐velocity (JointVelocityCmd) toward that angle.

    Commands wheel joints to spin at the requested speed (from sqrt(linearX² + linearY²)).

Usage

    Include in your robot’s SDF/URDF:

    <gazebo>
        <plugin name="crab_steering" filename="libcrab_steering_plugin.so"/>
    </gazebo>

Ensure your GAZEBO_PLUGIN_PATH (or IGN_GAZEBO_SYSTEM_PLUGIN_PATH) includes:

    ~/nav2_ws/install/crab_steering_plugin/lib

Launch Gazebo with your world/URDF and send /cmd_vel via ROS 2.

Plugin Parameters (SDF)

In this version PID gains are hardcoded in CrabSteeringPlugin.hpp.
To expose them via SDF, add elements under the <plugin> tag and parse in Configure().

Package: drive
What It Does

    Provides a robot_description (URDF) for a six‑wheeled “drive” model with IMU & LiDAR.

    Launch files to spawn the robot in an Ignition Gazebo world.

    A simple C++ subscriber.cpp that listens to /cmd_vel and prints the values.

    A Python drive_node.py placeholder for future robot control.

Usage

    Launch Gazebo:

    ros2 launch drive gazebo.launch.py

    This will:

    Start robot_state_publisher, joint_state_publisher

    Spawn the robot in worlds/maze.sdf via ros_gz_sim

    Display Visualization:

    ros2 launch drive display.launch.py

    Brings up RViz2 with the robot’s TF & sensors.

    Launching the Robot

    gazebo.launch.py — spawns your drive.urdf in maze.sdf.

    display.launch.py — runs RViz2 to show TF & LaserScan.

Testing with cmd_vel

    Run the C++ subscriber:

    ros2 run drive subscriber

    Publish through drive_node

    ros2 run drive_node

    Use WASD to control crab steered 6 rover.

    PID Tuning

The default PID gains in CrabSteeringPlugin.hpp:

pid.kp = 5.0;
pid.ki = 0.0;
pid.kd = 0.2;

    Increase kp for faster response.

    Increase kd to damp oscillations.

    Add ki only if steady‐state error persists.

    License

This project is released under the MIT License — see LICENSE for details.

NOTE:

Add to .bashrc or .zshrc:

source ~/nav2_ws/install/setup.bash

export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins:~/nav2_ws/install/crab_steering_plugin/lib:$GAZEBO_PLUGIN_PATH
export IGN_GAZEBO_PLUGIN_PATH=~/nav2_ws/install/crab_steering_plugin/lib:$IGN_GAZEBO_PLUGIN_PATH
export IGN_GAZEBO_RESOURCE_PATH=~/nav2_ws/src/drive/models:$IGN_GAZEBO_RESOURCE_PATH
