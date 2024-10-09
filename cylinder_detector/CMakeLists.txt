cmake_minimum_required(VERSION 3.5)
project(cylinder_detector)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Add the executable
add_executable(cylinder_detector_node src/cylinder_detector_node.cpp)

# Link against OpenCV libraries
target_link_libraries(cylinder_detector_node ${OpenCV_LIBS})

# Specify dependencies for the executable
ament_target_dependencies(cylinder_detector_node rclcpp sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs OpenCV)

# Install the executable
install(TARGETS
  cylinder_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# ament package
ament_package()

