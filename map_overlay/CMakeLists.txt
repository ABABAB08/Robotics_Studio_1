cmake_minimum_required(VERSION 3.5)
project(map_overlay)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)

# Add the executable
add_executable(overlay_maps_node src/overlay_maps.cpp)

# Link against libraries
target_link_libraries(overlay_maps_node ${OpenCV_LIBRARIES})
ament_target_dependencies(overlay_maps_node rclcpp)

# Install
install(TARGETS
  overlay_maps_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

