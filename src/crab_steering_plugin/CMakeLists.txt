cmake_minimum_required(VERSION 3.8)
project(crab_steering_plugin)

# Add compiler options if using GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Remove direct CMAKE_PREFIX_PATH manipulation
# set(CMAKE_PREFIX_PATH "/opt/ros/humble")
# list(APPEND CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH})

# Find necessary packages
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ros_gz_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)      # Add this
find_package(ros_gz_bridge REQUIRED)   # Add this

# Define the plugin as a shared library
add_library(crab_steering_plugin SHARED
  src/CrabSteeringPlugin.cpp
)

# Specify the include directories for the plugin
target_include_directories(crab_steering_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Link dependencies to the plugin
target_link_libraries(crab_steering_plugin
  ${rclcpp_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)

# Link dependencies to the plugin
ament_target_dependencies(crab_steering_plugin
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  tf2_ros
  ignition-gazebo6
  ros_gz_sim
  ros_gz_bridge
)

ros_gz_create_plugin_library(
  ${PROJECT_NAME} ${PLUGIN_NAME}
  MODULE
  ${${PROJECT_NAME}_SRCS}
  DEPENDENCIES "rclcpp rclpy geometry_msgs nav_msgs tf2_ros ros_gz_sim ros_gz_bridge" # Use ros_gz_* here
)

# Install the plugin library
install(TARGETS crab_steering_plugin
  LIBRARY DESTINATION lib
)

# For testing, use linting and other tools (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()