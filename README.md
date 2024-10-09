# Project Overview
This project utilizes several ROS 2 packages for robotic functionalities. The main packages used for Sprint 3 include:

- **cylinder_detector**: For detecting cylindrical objects in the environment.
- **map_overlay**: For overlaying different map data to visualize alignment and offsets.

## Packages Included

### 1. cylinder_detector
This package processes laser scan data and detects cylindrical objects of specified dimensions. It uses laser scan information to identify objects that match certain size and shape criteria (e.g., a 30 cm diameter cylinder). The node publishes the detected object's position and size, which can be visualized or used in other ROS nodes.

### 2. map_overlay
This package allows for overlaying different maps (e.g., a ground truth map and a generated map) for comparison and visualization. It uses OpenCV to perform the overlay, and users can adjust the alignment using trackbars to find the best match between maps. This visualization helps to assess how well the maps align with each other.

## Running the Project
To run the project, ensure you have the dependencies installed and sourced. Use the following commands to launch the nodes:

```bash
ros2 run cylinder_detector cylinder_detector_node
ros2 run map_overlay map_overlay_node
```

These commands will start the object detection and map overlay functionalities. Make sure that the required map files are in the specified directories for the map overlay node to function correctly.

## Requirements
- ROS 2 Humble or newer
- OpenCV 4.2 or newer
- rclcpp

## Installation
Clone the project repository to your workspace.

Build the project using colcon:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

## File Structure
```
src/
├── cylinder_detector/
│   ├── src/          # Source code for ROS node.
│   └── launch/       # Launch files for starting the node.
└── map_overlay/
    ├── src/          # Source code for ROS node.
    └── launch/       # Launch files for starting the node.
```

## Usage Notes
- The `cylinder_detector` node processes laser scan data and detects objects based on the configured parameters for object size.
- The `map_overlay` node allows for interactive map alignment using trackbars. Adjust the offsets to align the maps.

## Author
[Anthony Biscotto]


## Additional Resources
- [ROS 2 Documentation](https://docs.ros.org/en/)
- [OpenCV Documentation](https://docs.opencv.org/)
