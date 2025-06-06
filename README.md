# 6-Wheeled Rover with Differential Steering and Teleop Control

This package provides a complete simulation and control system for a 6-wheeled rover with ackermann steering geometry and differential drive capabilities.

## Features

- ✅ **6-Wheeled Rover Simulation**: Complete Gazebo simulation with realistic physics
- ✅ **Differential Steering**: Ackermann steering geometry for all 6 wheels
- ✅ **Teleop Control**: Keyboard and joystick control support  
- ✅ **ROS2 Control Integration**: Full ros2_control hardware abstraction
- ✅ **Sensor Integration**: IMU and LiDAR sensors with ROS2 bridges
- ✅ **Real-time Monitoring**: Status monitoring and diagnostics

## Quick Start


### 1. Manual Keyboard Control

If you want just teleop without full simulation:

```bash
cd ~/nav2_ws
source install/setup.bash

# Terminal 1: Launch simulation
ros2 launch drive rover_complete.launch.py

# Terminal 2: Start teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```

### 3. Keyboard Controls

Once the teleop terminal opens, use these keys:
⚙️ Speed Controls

Speed Adjustment:
- q : Increase max speeds by 10%
- z : Decrease max speeds by 10%
- w : Increase only linear speed by 10%
- x : Decrease only linear speed by 10%
- e : Increase only angular speed by 10%
- c : Decrease only angular speed by 10%

Stop & Exit:
- Any other key : Emergency stop
- Ctrl+C : Quit teleop

🚗 For Your 6-Wheeled Rover

Most Useful Keys:
- i : Drive forward
- , : Drive backward
- j : Turn left (differential steering)
- l : Turn right (differential steering)  
- u : Forward while turning left
- o : Forward while turning right
- k : Complete stop
- w/x : Adjust forward/backward speed
- e/c : Adjust turning speed

Default Speeds:
- Linear speed: 0.5 m/s
- Angular speed: 1.0 rad/s


   CTRL+C : Exit teleop
```

## Command Examples

### Direct ROS2 Commands

You can also send commands directly via ROS2 topics:

```bash
# Forward motion
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Turn in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'

# Forward with steering
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## System Architecture

```
Laptop Keyboard/Joystick
         ↓
   teleop_twist_keyboard
         ↓
      /cmd_vel topic
         ↓
ackermann_cmd_vel_converter  (differential_steering package)
         ↓
/drive_controller/commands + /steer_controller/commands
         ↓
    ros2_control system
         ↓
    Gazebo Simulation
```

## Monitoring and Debugging

### Check System Status

```bash
# Check running nodes
ros2 node list

# Check topics
ros2 topic list

# Monitor cmd_vel commands
ros2 topic echo /cmd_vel

# Monitor wheel commands
ros2 topic echo /drive_controller/commands

# Monitor steering commands  
ros2 topic echo /steer_controller/commands

# Check joint states
ros2 topic echo /joint_states
```

### Run Status Monitor

```bash
cd ~/nav2_ws
source install/setup.bash
python3 src/drive/scripts/rover_controller.py
```



## Launch File Options


2. **rover_complete.launch.py**: Full simulation without teleop
3. **gazebo.launch.py**: Basic Gazebo simulation only
4. **teleop_only.launch.py**: Just teleop for existing simulation

## Troubleshooting

### Common Issues

1. **Rover doesn't move**: Check if controllers are loaded
   ```bash
   ros2 control list_controllers
   ```

2. **No teleop response**: Ensure terminal has focus and correct remapping
   ```bash
   ros2 topic echo /cmd_vel  # Should show messages when keys pressed
   ```

3. **Gazebo crashes**: Ensure ign_ros2_control is installed
   ```bash
   sudo apt install ros-humble-ign-ros2-control
   ```


