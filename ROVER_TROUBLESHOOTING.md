# Rover Teleop Troubleshooting Guide

## Issues Fixed

### 1. Sensor Bridge Configuration
**Problem**: IMU and LiDAR sensors were not publishing data due to incorrect topic mapping in the bridge configuration.

**Solution**: 
- Fixed IMU bridge: Changed from `/imu/data` to `/imu` in Gazebo, then remapped to `/imu/data` in ROS2
- Fixed LiDAR bridge: Changed from `/scan` to `/lidar` in Gazebo, then remapped to `/scan` in ROS2
- Updated URDF sensor topic configuration to match

### 2. Controller Loading
**Problem**: Controllers appeared to be failing to load.

**Solution**: 
- Verified that both `drive_controller` and `steer_controller` are actually loading correctly
- The ros2_control system is working properly with the IgnitionSystem plugin

## Current Status

✅ **Working Components:**
- Robot spawning in Gazebo
- ros2_control system with drive and steering controllers
- Robot state publisher
- Joint state broadcaster
- Differential steering converter
- Rover status monitor

✅ **Fixed Components:**
- Sensor bridge configuration
- IMU sensor topic mapping
- LiDAR sensor topic mapping

## How to Test

1. **Launch the rover:**
   ```bash
   cd /home/ujjwal/nav2_ws
   source install/setup.bash
   ros2 launch drive rover_teleop.launch.py
   ```

2. **Verify controllers are loaded:**
   ```bash
   ros2 control list_controllers
   ```
   Should show:
   - drive_controller (active)
   - steer_controller (active)
   - joint_state_broadcaster (active)

3. **Check available topics:**
   ```bash
   ros2 topic list
   ```
   Should include:
   - `/cmd_vel` (when teleop is running)
   - `/imu/data`
   - `/scan`
   - `/joint_states`
   - `/drive_controller/commands`
   - `/steer_controller/commands`

4. **Test rover movement:**
   ```bash
   # In a new terminal
   cd /home/ujjwal/nav2_ws
   source install/setup.bash
   python3 src/drive/scripts/test_rover.py
   ```

5. **Manual teleop control:**
   ```bash
   # In a new terminal
   cd /home/ujjwal/nav2_ws
   source install/setup.bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
   ```

## Key Configuration Files

1. **Launch file**: `src/drive/launch/rover_teleop.launch.py`
2. **URDF**: `src/drive/models/drive/urdf/drive.urdf`
3. **Controller config**: `src/drive/models/drive/config/rover_controllers.yaml`
4. **Status monitor**: `src/drive/scripts/rover_status_monitor.py`
5. **Test script**: `src/drive/scripts/test_rover.py`

## Troubleshooting Commands

```bash
# Check controller status
ros2 control list_controllers

# Check hardware interfaces
ros2 control list_hardware_interfaces

# Monitor joint states
ros2 topic echo /joint_states

# Monitor controller commands
ros2 topic echo /drive_controller/commands
ros2 topic echo /steer_controller/commands

# Check sensor data
ros2 topic echo /imu/data
ros2 topic echo /scan

# Test cmd_vel publishing
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'
```

## Expected Behavior

When the launch file runs successfully:
1. Gazebo opens with the maze world
2. The 6-wheeled rover spawns at position (0, 0, 0.7)
3. Controllers load and become active
4. Sensor bridges start publishing IMU and LiDAR data
5. A terminal opens for keyboard teleop control
6. The rover status monitor provides real-time feedback

The rover should respond to teleop commands and move in the simulation environment.