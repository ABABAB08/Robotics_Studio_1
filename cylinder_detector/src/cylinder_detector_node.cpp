#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

/**
 * @class ObjDetectionNode
 * @brief A ROS2 node for detecting cylindrical objects using laser scan data.
 *
 * This class subscribes to laser scan and odometry topics, processes the laser scan data
 * to detect cylindrical objects based on their diameter, and displays the results on a map.
 */
class ObjDetectionNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor for ObjDetectionNode.
     *
     * Initializes the node, sets up subscriptions, loads the map, and creates display windows.
     */
    ObjDetectionNode()
    : Node("object_detection_node") {
        // Subscribe to the laser scan and odometry topics
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                            "/scan", 10, std::bind(&ObjDetectionNode::scanCallback, this, std::placeholders::_1));
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                            "/odom", 10, std::bind(&ObjDetectionNode::odomCallback, this, std::placeholders::_1));

        // Load the map image
        gazebo_map_ = cv::imread("/home/student/Desktop/Robotics_Studio_1/Robotics_Studio_1/src/my_worlds_pkg/mapping/project_map.pgm");
        original_gazebo_map_ = gazebo_map_.clone();

        // Set map resolution
        map_scale_ = 0.05;

        // Create display windows
        cv::namedWindow(WINDOW_Map, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW_LaserScan, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(WINDOW_ObjDetect, cv::WINDOW_AUTOSIZE);

        cv::imshow(WINDOW_Map, gazebo_map_);
        cv::waitKey(1);
    }

private:
    /**
     * @brief Callback function for processing laser scan data.
     *
     * Converts the laser scan data into an OpenCV image and performs object detection.
     * @param scanMsg The received laser scan message.
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scanMsg) {
        scan = *scanMsg;
        cv::Mat laser_image = laserScanToMat(scanMsg);
        cv::rotate(laser_image, laser_image, cv::ROTATE_90_COUNTERCLOCKWISE);

        cv::imshow(WINDOW_LaserScan, laser_image);
        cv::waitKey(1);

        objDetection(scan);
    }

    /**
     * @brief Callback function for processing odometry data.
     *
     * Updates the robot's current position and orientation based on the odometry message.
     * @param odomMsg The received odometry message.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odomMsg) {
        odom_ = *odomMsg;
    }

    /**
     * @brief Performs object detection on the laser scan data.
     *
     * This function identifies cylindrical objects within a specified size range using laser scan data.
     * If an object is detected, it is marked on the map.
     * @param Scan The laser scan data.
     */
    void objDetection(const sensor_msgs::msg::LaserScan Scan) {
        // Set parameters for cylindrical object detection
        float object_diameter = 0.30;  // Diameter in meters
        float tolerance = 0.07;        // Tolerance in meters
        float min_radius = (object_diameter - tolerance) / 2.0;  // Minimum radius in meters
        float max_radius = (object_diameter + tolerance) / 2.0;  // Maximum radius in meters

        // Define range limits
        float min_distance = Scan.range_min;
        float max_distance = 0.2 * Scan.range_max;

        std::vector<cv::Point2f> laser_points;

        // Extract the robot's position and orientation from odometry
        float robot_x = odom_.pose.pose.position.x;
        float robot_y = odom_.pose.pose.position.y;

        // Convert orientation from quaternion to yaw
        tf2::Quaternion quat(
            odom_.pose.pose.orientation.x,
            odom_.pose.pose.orientation.y,
            odom_.pose.pose.orientation.z,
            odom_.pose.pose.orientation.w
        );
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        // Collect valid points from the laser scan
        for (size_t i = 0; i < Scan.ranges.size(); ++i) {
            float range = Scan.ranges[i];
            if (range > min_distance && range < max_distance) {
                float angle = Scan.angle_min + i * Scan.angle_increment;

                // Convert polar to Cartesian coordinates
                float local_x = range * cos(angle);
                float local_y = range * sin(angle);

                // Transform to global coordinates
                float global_x = robot_x + (local_x * cos(yaw) - local_y * sin(yaw));
                float global_y = robot_y + (local_x * sin(yaw) + local_y * cos(yaw));

                laser_points.push_back(cv::Point2f(global_x, global_y));
            }
        }

        if (laser_points.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "Not enough points detected.");
            return;
        }

        // Find the minimum enclosing circle for the detected points
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(laser_points, center, radius);

        // Check if the detected circle matches the expected size
        if (radius >= min_radius && radius <= max_radius) {
            RCLCPP_INFO(this->get_logger(), "Detected cylindrical object with radius: %f", radius);
            RCLCPP_INFO(this->get_logger(), "Object center coordinates: x = %f, y = %f", center.x, center.y);

            // Reset the gazebo map
            gazebo_map_ = original_gazebo_map_.clone();

            // Convert object coordinates to map scale
            int map_x = static_cast<int>((center.x / map_scale_) + (gazebo_map_.cols / 2));
            int map_y = static_cast<int>((-center.y / map_scale_) + (gazebo_map_.rows / 2));

            // Draw a circle to represent the detected object
            int circle_radius_in_map = static_cast<int>((object_diameter / 2.0) / map_scale_);
            cv::circle(gazebo_map_, cv::Point(map_x, map_y), circle_radius_in_map, cv::Scalar(0, 255, 0), 2);

            // Display the map with the marked object
            cv::imshow(WINDOW_ObjDetect, gazebo_map_);
            cv::waitKey(1);
        }
    }

    /**
     * @brief Converts laser scan data to an OpenCV image.
     *
     * This function converts the laser scan data into a 2D grayscale image.
     * @param scan The laser scan message.
     * @return An OpenCV image representing the laser scan.
     */
    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        int img_size = 500;
        float max_range = scan->range_max;
        cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            if (range > scan->range_min && range < scan->range_max) {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
                int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;
                if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                    image.at<uchar>(y, x) = 255;
                }
            }
        }
        return image;
    }

    // ROS subscriptions
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    sensor_msgs::msg::LaserScan scan;
    nav_msgs::msg::Odometry odom_;

    // Map data
    cv::Mat gazebo_map_;
    cv::Mat original_gazebo_map_;

    double map_scale_;

    // Window names
    const std::string WINDOW_Map = "Map";
    const std::string WINDOW_LaserScan = "Laser Scan";
    const std::string WINDOW_ObjDetect = "Detected Object";
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
    rclcpp::spin(std::make_shared<ObjDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
