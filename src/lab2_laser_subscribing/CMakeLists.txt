cmake_minimum_required(VERSION 3.5)
project(lab2_laser_subscribing)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add executable
add_executable(laser_scan_processor src/laser_scan_processor.cpp)

# Specify dependencies of the executable target
ament_target_dependencies(laser_scan_processor rclcpp sensor_msgs)

# Install executable
install(TARGETS
  laser_scan_processor
  DESTINATION lib/${PROJECT_NAME})

# Ament package
ament_package()
