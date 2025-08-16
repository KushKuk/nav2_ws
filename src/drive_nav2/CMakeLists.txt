cmake_minimum_required(VERSION 3.8)
project(drive_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(gz_ros2_control REQUIRED)

# Install directories
install(
  DIRECTORY
    config
    launch
    urdf
    worlds
    models
    meshes
    rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts and make them executable
install(
  PROGRAMS
    scripts/follow_waypoints.py
    scripts/reach_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following ensures that ament_lint_cmake is run
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()