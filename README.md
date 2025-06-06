Six-Wheeled Rover Simulation with ROS 2 and Ignition Gazebo
This project simulates a six-wheeled rover with Ackermann-like steering in Ignition Gazebo, controlled via ROS 2. The rover navigates a maze environment (maze.sdf), using ros2_control for wheel and steering joint control, and supports keyboard teleoperation via drive_node.py. The simulation includes IMU and LiDAR sensors for potential navigation tasks.
Project Structure

URDF: drive/models/drive/urdf/drive.urdf
Defines the rover with six wheels, steering joints, IMU, and LiDAR.
Includes gazebo_ros2_control and JointStatePublisher plugins.


World: drive/worlds/maze.sdf
A maze with static walls and a ground plane for navigation.


Launch: drive/launch/ros2_control.launch.py
Launches Gazebo, spawns the rover, initializes controllers, and runs teleoperation.


Config: drive/models/drive/config/rover_controllers.yaml
Configures drive_controller (wheel velocities) and steer_controller (steering angles).


Scripts:
drive/scripts/subscriber.cpp: A ROS 2 node to print /cmd_vel messages for debugging.
drive/scripts/drive_node.py: A keyboard teleoperation node to publish /cmd_vel.


Package: drive (ROS 2 package in ~/nav2_ws/src/drive).

Prerequisites

OS: Ubuntu 22.04 (Jammy) or later.
ROS 2: Humble Hawksbill (recommended).
Ignition Gazebo: Fortress (compatible with ros_gz_sim).
Dependencies:
ros2_control, ros2_controllers, gazebo_ros2_control
ros_gz_sim, ros_gz_bridge
robot_state_publisher
differential_steering (for ackermann_cmd_vel_converter)
teleop_twist_keyboard (optional, if drive_node.py is unavailable)



Install dependencies:
sudo apt update
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
sudo apt install ros-humble-robot-state-publisher ros-humble-teleop-twist-keyboard

Setup

Clone Repository:
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws/src
git clone <repository_url> drive


Update maze.sdf:

Add physics parameters to drive/worlds/maze.sdf:<world name="maze_world">
  <physics>
    <real_time_update_rate>50</real_time_update_rate>
    <max_step_size>0.02</max_step_size>
  </physics>
  ...
</world>




Verify drive_node.py:

Ensure drive_node.py is in drive/scripts/ and listed in drive/setup.py:from setuptools import setup
...
setup(
    ...
    scripts=['scripts/drive_node.py'],
    ...
)


If unavailable, use teleop_twist_keyboard by modifying drive/launch/ros2_control.launch.py:teleop_node = Node(
    package='teleop_twist_keyboard',
    executable='teleop_twist_keyboard',
    name='teleop_twist_keyboard',
    parameters=[{'use_sim_time': True}],
    output='screen',
    remappings=[('cmd_vel', '/cmd_vel')]
)




Build Workspace:
cd ~/nav2_ws
colcon build
source install/setup.bash



Usage

Launch Simulation:
ros2 launch drive ros2_control.launch.py


Starts Ignition Gazebo with maze.sdf.
Spawns the rover at (0, 0, 0.7) in the start room.
Initializes ros2_control with drive_controller and steer_controller.
Runs ackermann_cmd_vel_converter to process /cmd_vel.
Launches drive_node.py for keyboard control.


Control Rover:

Use drive_node.py’s keyboard bindings (e.g., arrow keys for linear.x, angular.z).
If using teleop_twist_keyboard:
i: Move forward (linear.x > 0).
,: Move backward (linear.x < 0).
j: Turn left (angular.z > 0).
l: Turn right (angular.z < 0).


Monitor /cmd_vel:ros2 topic echo /cmd_vel




Debug with Subscriber:
ros2 run drive subscriber


Prints linear.x and angular.z from /cmd_vel.


Monitor Outputs:
ros2 topic echo /drive_controller/commands  # Wheel velocities
ros2 topic echo /steer_controller/commands  # Steering angles
ros2 topic echo /joint_states               # Joint states
ros2 topic echo /scan                      # LiDAR data
ros2 topic echo /imu/data                  # IMU data


Visualize in RViz:
ros2 launch drive display.launch.py


Displays the rover model, joint states, LiDAR scans, and IMU data.



Control Pipeline

Input: drive_node.py publishes geometry_msgs::msg::Twist on /cmd_vel (linear.x, angular.z).
Conversion: ackermann_cmd_vel_converter translates /cmd_vel to:
/drive_controller/commands (std_msgs::msg::Float64MultiArray): Wheel angular velocities (rad/s).
/steer_controller/commands: Steering angles (radians, ±1.047 rad max).


Control: ros2_control (via gazebo_ros2_control plugin) applies commands to joint_wheel_* and joint_rotate_* joints.
Feedback: JointStatePublisher plugin publishes /joint_states for visualization and navigation.
Sensors: IMU (/imu/data) and LiDAR (/scan) provide data for potential navigation tasks.

Troubleshooting

Rover Doesn’t Move:

Check Gazebo logs for plugin errors.
Verify /drive_controller/commands has non-zero values:ros2 topic echo /drive_controller/commands


Ensure ros2_control_node activates controllers (check terminal output).
Confirm use_sim_time is set in ros2_control.launch.py and rover_controllers.yaml.


Pylance Errors:

If drive_urdf is undefined, ensure urdf_path is used correctly in ros2_control.launch.py (see fixed version).


Keyboard Control Issues:

Verify drive_node.py is executable and publishes to /cmd_vel.
Use ros2 run teleop_twist_keyboard teleop_twist_keyboard as a fallback.
Run ros2 run drive subscriber to debug /cmd_vel.


Simulation Jitter:

Confirm maze.sdf has <real_time_update_rate>50</real_time_update_rate> and <max_step_size>0.02</max_step_size>.


Joint Order:

If steering or wheel commands are incorrect, check joint_names_drive.yaml (if used) matches drive.urdf order: [front left, front right, middle left, middle right, rear left, rear right].



Future Work

Autonomous Navigation: Integrate Nav2 for maze navigation using /scan, /imu/data, /joint_states, and odometry.
Odometry: Add an odometry publisher (e.g., robot_localization) for pose estimation.
Custom Teleop: Enhance drive_node.py with adjustable speed or additional controls.

License
This project is licensed under the MIT License. See the LICENSE file for details.
Contact
For issues or contributions, open a GitHub issue or contact the repository maintainer.
