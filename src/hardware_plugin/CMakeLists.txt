cmake_minimum_required(VERSION 3.8)
project(hardware_plugin)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Build shared library (plugin)
add_library(${PROJECT_NAME} SHARED
  src/hardware_plugin.cpp
)

# Include headers
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  pluginlib
  hardware_interface
  sensor_msgs
  nav_msgs
)

# Export the plugin to pluginlib
pluginlib_export_plugin_description_file(hardware_plugin hardware_plugin.xml)

# Install targets and headers
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
