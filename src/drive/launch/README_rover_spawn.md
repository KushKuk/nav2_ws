# Rover Spawn Launch File

This launch file (`new_rover_spawn.launch.py`) spawns the 6-wheeled rover in the maze world simulation.

## Usage

To launch the rover in the maze world:

```bash
cd ~/nav2_ws
source install/setup.bash
ros2 launch drive new_rover_spawn.launch.py
```

## Launch Arguments

You can customize the spawn parameters:

```bash
ros2 launch drive new_rover_spawn.launch.py \
    robot_name:=my_rover \
    spawn_x:=0.0 \
    spawn_y:=-8.0 \
    spawn_z:=0.5 \
    spawn_yaw:=1.57
```

### Available Arguments:
- `robot_name`: Name of the robot (default: "drive_rover")
- `spawn_x`: X position to spawn the robot (default: 0.0)
- `spawn_y`: Y position to spawn the robot (default: -8.0, start area)
- `spawn_z`: Z position to spawn the robot (default: 0.5)
- `spawn_yaw`: Yaw angle to spawn the robot (default: 1.57, facing forward)
- `use_sim_time`: Use simulation time (default: true)

## What it does:

1. Starts Ignition Gazebo with the maze world
2. Publishes the robot description from the URDF file
3. Spawns the 6-wheeled rover at the specified position
4. Sets up ROS-Gazebo bridges for:
   - Clock synchronization
   - Velocity commands (`/cmd_vel`)
   - Odometry (`/odom`)
   - Transform frames (`/tf`)
   - LiDAR data (`/scan`)
   - IMU data (`/imu`)
5. Starts joint state publisher for the rover

## Controlling the Rover

After launching, you can control the rover using:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Or monitor the topics:
```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /scan
```