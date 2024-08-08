#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
  ImageSubscriber()
  : Node("image_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&ImageSubscriber::topic_callback, this, std::placeholders::_1));
    
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_with_circle", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Convert ROS Image message to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Get image dimensions
    int height = cv_ptr->image.rows;
    int width = cv_ptr->image.cols;

    // Define the center and radius of the circle
    cv::Point center(width / 2, height / 2);
    int radius = std::min(width, height) / 10;

    // Draw the circle
    cv::circle(cv_ptr->image, center, radius, CV_RGB(0, 255, 0), 3);

    // Convert the modified image back to ROS Image message
    auto image_msg = cv_ptr->toImageMsg();
    publisher_->publish(*image_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}
