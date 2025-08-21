cmake_minimum_required(VERSION 3.10)
project(drive)

# Ensure you find the necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(controller_manager REQUIRED)
find_package(velocity_controllers REQUIRED)
find_package(position_controllers REQUIRED)


# Install directories for launch, worlds, and models
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install Python script (e.g., drive_node.py)
install(PROGRAMS
  scripts/drive_node.py
  DESTINATION lib/${PROJECT_NAME}
)






# Export dependencies
ament_package()

# Install additional Python scripts for rover control
install(PROGRAMS
  scripts/odometry_publisher.py
  scripts/drive_node.py
  scripts/rover_controller.py
  scripts/test_rover_movement.py
  scripts/rover_status_monitor.py
  scripts/test_differential_steering.py
  scripts/rover_diagnostics.py
  DESTINATION lib/${PROJECT_NAME}
)