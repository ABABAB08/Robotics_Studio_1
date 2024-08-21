#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserScanProcessor : public rclcpp::Node
{
public:
    LaserScanProcessor()
    : Node("laser_scan_processor")
    {
        // Subscriber to the /scan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanProcessor::scanCallback, this, std::placeholders::_1));

        // Publisher for the filtered scan
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/filtered_scan", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Example: Get the range at the middle angle
        size_t angle_index = msg->ranges.size() / 2;
        float range_at_angle = msg->ranges[angle_index];
        RCLCPP_INFO(this->get_logger(), "Range at angle: %.2f meters", range_at_angle);

        // Create a new LaserScan message for filtered data
        auto filtered_scan = std::make_shared<sensor_msgs::msg::LaserScan>();

        // Copy the header and time-related parameters
        filtered_scan->header = msg->header;
        filtered_scan->time_increment = msg->time_increment;
        filtered_scan->scan_time = msg->scan_time;
        filtered_scan->range_min = msg->range_min;
        filtered_scan->range_max = msg->range_max;

        // Select a subset of range values (for example, 90 degrees centered around the middle)
        size_t total_angles = msg->ranges.size();
        size_t start_index = total_angles / 4; // Start at 45 degrees
        size_t end_index = 3 * total_angles / 4; // End at 135 degrees

        filtered_scan->ranges.assign(msg->ranges.begin() + start_index, msg->ranges.begin() + end_index);

        // Adjust angle_min and angle_max according to the subset
        filtered_scan->angle_min = msg->angle_min + start_index * msg->angle_increment;
        filtered_scan->angle_max = msg->angle_min + end_index * msg->angle_increment;
        filtered_scan->angle_increment = msg->angle_increment;

        // Publish the filtered scan
        publisher_->publish(*filtered_scan);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserScanProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
