cmake_minimum_required(VERSION 3.5)
project(opencv_lab1)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Add executable
add_executable(image_subscriber src/image_subscriber.cpp)

# Specify dependencies of the executable target
ament_target_dependencies(image_subscriber rclcpp sensor_msgs cv_bridge OpenCV)

# Install executable
install(TARGETS
  image_subscriber
  DESTINATION lib/${PROJECT_NAME})

# Ament package
ament_package()
