# Controller Spawning Debug Guide - SOLVED!

## Issues Fixed:

1. **Added missing `name` attribute** to `<ros2_control>` tag in URDF
2. **Corrected plugin name** to `ign_ros2_control/IgnitionSystem`
3. **Added Gazebo plugin** for ros2_control integration
4. **Removed duplicate ros2_control_node** - let Gazebo handle it
5. **Fixed wheel radius parameter** to use consistent value (0.1125) across all launch files
6. **Added proper timing delays** for controller spawning

## Root Cause:
The main issue was that the URDF was missing the required `name` attribute in the `<ros2_control>` tag and the wrong plugin name was being used. Additionally, the approach of running a separate `ros2_control_node` was conflicting with Gazebo's built-in ros2_control integration.

## Solution Applied:

### URDF Changes (drive.urdf):
- Added `name="RoverHardware"` to `<ros2_control>` tag
- Changed plugin from `gz_ros2_control/GazeboSystem` to `ign_ros2_control/IgnitionSystem`
- Added Gazebo plugin section for ros2_control integration

### Launch File Changes:
- **ros2_control.launch.py**: Removed separate ros2_control_node, now only spawns controllers
- **rover_teleop.launch.py**: Fixed wheel_radius parameter consistency

## Verification - Controllers Now Working:

```bash
$ ros2 control list_controllers
drive_controller velocity_controllers/JointGroupVelocityController  active
steer_controller position_controllers/JointGroupPositionController  active
```

## Testing Your Fixed Setup:

1. **Launch your rover:**
   ```bash
   cd /home/ujjwal/nav2_ws
   source install/setup.bash
   ros2 launch drive rover_teleop.launch.py
   ```

2. **Verify controllers are active:**
   ```bash
   ros2 control list_controllers
   ```
   Both controllers should show as "active"

3. **Test teleop control** - The gnome-terminal should open automatically with keyboard teleop

4. **Verify controller topics:**
   ```bash
   ros2 topic list | grep -E 'drive_controller|steer_controller'
   ```
   Should show:
   - /drive_controller/commands
   - /steer_controller/commands

## Controller Configuration Summary:

- **drive_controller**: Controls 6 wheel joints (joint_wheel_1 to joint_wheel_6) with velocity commands ✅
- **steer_controller**: Controls 4 steering joints (joint_right_front, joint_right_back, joint_left_front, joint_left_back) with position commands ✅
- **ackermann_cmd_vel_converter**: Converts /cmd_vel to controller commands with proper 6-wheel rover kinematics ✅

## Success Indicators:
- ✅ No ros2_control_node crashes
- ✅ Controllers spawn and activate successfully
- ✅ Teleop keyboard control works
- ✅ Rover responds to movement commands in Gazebo

## Issues Fixed:

1. **Removed duplicate ackermann_cmd_vel_converter** from ros2_control.launch.py
2. **Added proper timing delays** for controller spawning (5s for drive_controller, 6s for steer_controller)
3. **Fixed wheel radius parameter** to use consistent value (0.1125) across all launch files
4. **Replaced hardcoded paths** with proper package share directory paths

## Changes Made:

### ros2_control.launch.py:
- Removed duplicate ackermann_cmd_vel_converter node
- Added TimerAction delays for controller spawning
- Fixed hardcoded paths to use get_package_share_directory()
- Separated drive_controller and steer_controller spawning with different delays

### rover_teleop.launch.py:
- Fixed wheel_radius parameter from 0.28 to 0.1125 to match URDF specifications

## Testing Your Controllers:

1. **Launch the rover:**
   ```bash
   cd /home/ujjwal/nav2_ws
   source install/setup.bash
   ros2 launch drive rover_teleop.launch.py
   ```

2. **Check controller status (in a new terminal):**
   ```bash
   source /home/ujjwal/nav2_ws/install/setup.bash
   ros2 control list_controllers
   ```
   
   You should see:
   - drive_controller (active)
   - steer_controller (active)

3. **Verify controller topics:**
   ```bash
   ros2 topic list | grep -E 'drive_controller|steer_controller'
   ```
   
   Expected topics:
   - /drive_controller/commands
   - /steer_controller/commands

4. **Test manual controller commands:**
   ```bash
   # Test drive controller (wheel velocities)
   ros2 topic pub /drive_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]" --once
   
   # Test steer controller (steering angles)
   ros2 topic pub /steer_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, 0.5, 0.5, 0.5]" --once
   ```

## Common Issues and Solutions:

### If controllers still don't spawn:
1. Check Gazebo is running: `gz topic -l`
2. Verify ros2_control plugin: `ros2 topic list | grep /clock`
3. Check for error messages in the launch terminal

### If teleop doesn't work:
1. Verify ackermann_cmd_vel_converter is running: `ros2 node list | grep ackermann`
2. Check topic connections: `ros2 topic info /cmd_vel`
3. Test direct velocity commands: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5}" --once`

### If robot doesn't move in Gazebo:
1. Check joint states: `ros2 topic echo /joint_states`
2. Verify controller commands: `ros2 topic echo /drive_controller/commands`
3. Check Gazebo physics: Ensure the robot is not stuck or colliding

## Controller Configuration Summary:

- **drive_controller**: Controls 6 wheel joints (joint_wheel_1 to joint_wheel_6) with velocity commands
- **steer_controller**: Controls 4 steering joints (joint_right_front, joint_right_back, joint_left_front, joint_left_back) with position commands
- **ackermann_cmd_vel_converter**: Converts /cmd_vel to controller commands with proper 6-wheel rover kinematics