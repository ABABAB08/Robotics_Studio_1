#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

/**
 * @class MapOverlayNode
 * @brief A ROS2 node for overlaying two maps with adjustable offsets.
 *
 * This class loads two maps, aligns them using user-controlled offsets, and displays the overlay.
 * The user can adjust the overlay offsets through trackbars to achieve the desired alignment.
 */
class MapOverlayNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor for MapOverlayNode.
     *
     * Initializes the node, loads the maps, creates the overlay window, and sets up trackbars for offset adjustment.
     */
    MapOverlayNode()
        : Node("map_overlay_node"), offset_x_(50), offset_y_(50) {
        // Load the maps from the specified file paths
        gazebo_map_ = cv::imread("/home/student/Desktop/Robotics_Studio_1/Robotics_Studio_1/src/my_worlds_pkg/mapping/ground_truth/map.pgm", cv::IMREAD_GRAYSCALE);
        gmap_ = cv::imread("/home/student/Desktop/Robotics_Studio_1/Robotics_Studio_1/src/my_worlds_pkg/mapping/carto_map/sprint3_v1_carto_map.pgm", cv::IMREAD_GRAYSCALE);

        // Check if the maps were loaded successfully
        if (gazebo_map_.empty() || gmap_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load maps. Please check the file paths.");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Maps successfully loaded.");

        // Create a window to display the overlay
        cv::namedWindow("Map Overlay", cv::WINDOW_NORMAL);
        cv::resizeWindow("Map Overlay", 800, 800);

        // Create trackbars for offset adjustments
        cv::createTrackbar("Offset X", "Map Overlay", &offset_x_, gazebo_map_.cols - gmap_.cols, onTrackbarChange, this);
        cv::createTrackbar("Offset Y", "Map Overlay", &offset_y_, gazebo_map_.rows - gmap_.rows, onTrackbarChange, this);

        // Display the initial overlay
        updateOverlay();
        cv::waitKey(0);
    }

private:
    /**
     * @brief Trackbar callback function to update the overlay.
     *
     * This function is called whenever the trackbar values change.
     * It updates the overlay with the new offset values.
     * @param value The current value of the trackbar (not used).
     * @param userdata A pointer to the MapOverlayNode instance.
     */
    static void onTrackbarChange(int, void* userdata) {
        // Update the overlay when trackbar values change
        auto* node = static_cast<MapOverlayNode*>(userdata);
        node->updateOverlay();
    }

    /**
     * @brief Updates the map overlay based on the current offset values.
     *
     * Creates a new overlay image by combining the two maps with the specified offsets.
     * The overlay is then displayed in the "Map Overlay" window.
     */
    void updateOverlay() {
        // Create a blank canvas the size of the larger ground truth map
        cv::Mat aligned_map = cv::Mat::zeros(gazebo_map_.size(), gmap_.type());

        // Ensure the offset is within bounds
        int adjusted_offset_x = std::clamp(offset_x_, 0, gazebo_map_.cols - gmap_.cols);
        int adjusted_offset_y = std::clamp(offset_y_, 0, gazebo_map_.rows - gmap_.rows);

        // Place the GMapping map onto the blank canvas at the specified offset
        gmap_.copyTo(aligned_map(cv::Rect(adjusted_offset_x, adjusted_offset_y, gmap_.cols, gmap_.rows)));

        // Create an output image to hold the overlay
        cv::Mat overlay;
        cv::addWeighted(gazebo_map_, 0.5, aligned_map, 0.5, 0.0, overlay);

        // Display the overlay in the window
        cv::imshow("Map Overlay", overlay);
    }

    cv::Mat gazebo_map_; ///< The ground truth map (Gazebo map).
    cv::Mat gmap_;       ///< The GMapping map to be overlaid.
    int offset_x_;       ///< The horizontal offset for aligning the GMapping map.
    int offset_y_;       ///< The vertical offset for aligning the GMapping map.
};

/**
 * @brief Main function to start the ROS2 node.
 *
 * Initializes the ROS2 system, spins the node, and then shuts down.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return Exit status code.
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapOverlayNode>());
    rclcpp::shutdown();
    return 0;
}
