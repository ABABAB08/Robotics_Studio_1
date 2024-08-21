#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// Define a class that inherits from rclcpp::Node to create a ROS 2 node
class LaserScanSubPub : public rclcpp::Node
{
public:
    // Constructor for the LaserScanSubPub class
    LaserScanSubPub()
        : Node("laser_scan_subpub") // Initialize the node with a name
    {
        // Create a subscription to the /scan topic to receive LaserScan messages
        // The callback function 'scanCallback' will be called whenever a new message is received
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanSubPub::scanCallback, this, std::placeholders::_1));

        // Create a publisher that will publish the processed LaserScan messages on the /scan_nth topic
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_nth", 10);
    }

private:
    // Callback function that processes incoming LaserScan messages
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Create a new LaserScan message that will store the nth point data
        auto nth_scan = std::make_shared<sensor_msgs::msg::LaserScan>();

        // Copy the header and metadata from the original scan message
        nth_scan->header = scan->header;
        nth_scan->angle_min = scan->angle_min;
        nth_scan->angle_max = scan->angle_max;
        // Adjust the angle_increment and time_increment to account for the nth point selection
        nth_scan->angle_increment = scan->angle_increment * n_; // Multiply by 'n_' to skip points
        nth_scan->time_increment = scan->time_increment * n_;
        nth_scan->scan_time = scan->scan_time;
        nth_scan->range_min = scan->range_min;
        nth_scan->range_max = scan->range_max;

        // Loop through the original scan data, selecting every nth point
        for (size_t i = 0; i < scan->ranges.size(); i += n_)
        {
            // Add the nth range value to the new LaserScan message
            nth_scan->ranges.push_back(scan->ranges[i]);

            // If intensity data is available, add the nth intensity value as well
            if (!scan->intensities.empty())
            {
                nth_scan->intensities.push_back(scan->intensities[i]);
            }
        }

        // Publish the new LaserScan message containing only every nth point
        scan_pub_->publish(*nth_scan);
    }

    // Declare the subscription and publisher as private members of the class
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

    // Define the value of 'n_' to determine how many points to skip; e.g., n_ = 5 means every 5th point
    const int n_ = 5;  // Change this value to select a different nth point
};

// Main function that initializes and runs the node
int main(int argc, char *argv[])
{
    // Initialize the ROS 2 communication
    rclcpp::init(argc, argv);

    // Create an instance of the LaserScanSubPub node and keep it running
    rclcpp::spin(std::make_shared<LaserScanSubPub>());

    // Shutdown ROS 2 communication when the node stops running
    rclcpp::shutdown();
    return 0;
}
