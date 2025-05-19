cmake_minimum_required(VERSION 3.10)
project(drive)

# Required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(controller_manager REQUIRED)
find_package(velocity_controllers REQUIRED)
find_package(position_controllers REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Install launch files (only .py)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Install worlds
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install full models directory (urdf, meshes, config, textures, etc.)
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install Python scripts
install(PROGRAMS
  scripts/drive_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Add and install C++ node
add_executable(subscriber
  scripts/subscriber.cpp
)

ament_target_dependencies(subscriber
  rclcpp
  geometry_msgs
)

install(TARGETS subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies for other packages to find
ament_export_dependencies(
  rclcpp
  rclpy
  geometry_msgs
  controller_manager
  velocity_controllers
  position_controllers
  robot_state_publisher
  xacro
)

ament_package()
