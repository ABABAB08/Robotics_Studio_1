#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;

    // Create a blank image
    cv::Mat image = cv::Mat::zeros(400, 400, CV_8UC3);

    // Define the center and radius of the circle
    cv::Point center(200, 200);
    int radius = 50;

    // Draw the circle
    cv::circle(image, center, radius, cv::Scalar(0, 255, 0), 3);

    // Display the image
    cv::imshow("Test Image", image);
    cv::waitKey(0);

    return 0;
}
