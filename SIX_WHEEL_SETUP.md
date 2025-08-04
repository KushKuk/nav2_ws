# Six-Wheel Rover Controller - Quick Setup

Since there are some dependency issues with building the custom controller, here's a simple way to get your 6-wheeled rover working:

## Option 1: Direct Launch File (Recommended)

I've created a standalone launch file that you can run directly:

```bash
cd /home/ujjwal/nav2_ws
source /opt/ros/humble/setup.bash
ros2 launch six_wheel_rover_standalone.launch.py
```

## Option 2: Manual Controller Setup

If you prefer to set up the controllers manually:

### 1. Start the robot state publisher:
```bash
source /opt/ros/humble/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat /home/ujjwal/nav2_ws/src/drive/models/drive/urdf/drive.urdf)"
```

### 2. In another terminal, start the joint state broadcaster:
```bash
source /opt/ros/humble/setup.bash
ros2 run controller_manager spawner joint_state_broadcaster
```

### 3. In another terminal, start the differential drive controller:
```bash
source /opt/ros/humble/setup.bash
ros2 run controller_manager spawner diff_drive_controller --ros-args -p left_wheel_names:="[joint_wheel_1, joint_wheel_3, joint_wheel_5]" -p right_wheel_names:="[joint_wheel_2, joint_wheel_4, joint_wheel_6]" -p wheel_separation:=0.54 -p wheel_radius:=0.1125
```

## Control Your Rover

Once the controllers are running, you can control your rover with:

```bash
# Install teleop if not already installed
sudo apt install ros-humble-teleop-twist-keyboard

# Control the rover
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/diff_drive_controller/cmd_vel
```

## Configuration Details

Your 6-wheeled rover is configured as:
- **Left wheels**: joint_wheel_1, joint_wheel_3, joint_wheel_5
- **Right wheels**: joint_wheel_2, joint_wheel_4, joint_wheel_6
- **Wheel radius**: 0.1125m
- **Wheel separation**: 0.54m

## Troubleshooting

1. **"No robot_description parameter"**: Make sure the URDF file path is correct
2. **Controller not starting**: Check that the joint names in your URDF match the configuration
3. **No movement**: Verify that your hardware interfaces are properly configured

## Next Steps

For more advanced control (individual wheel steering), you would need to:
1. Implement proper hardware interfaces for your rover
2. Create custom kinematics for 6-wheel steering
3. Use a more sophisticated controller than differential drive

The current setup treats your 6-wheeled rover as a tank-drive system, which should work well for basic navigation.