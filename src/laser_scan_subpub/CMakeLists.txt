cmake_minimum_required(VERSION 3.5)
project(laser_scan_subpub)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Create the executable
add_executable(${PROJECT_NAME} src/laser_scan_subpub.cpp)

# Specify libraries against which to link
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
)

# Install the executable
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
