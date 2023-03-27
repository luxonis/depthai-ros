#include "depthai_bridge/ImageConverter.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"

// Inludes common necessary includes for development using depthai library

dai::rosBridge::ImageConverter inputConverter(true);

void rgbCallback(const sensor_msgs::msg::Image::SharedPtr rgbImageMsg) {
    cv::Mat rgbImage = inputConverter.rosMsgtoCvMat(*rgbImageMsg);
    cv::imshow("video", rgbImage);
    cv::waitKey(1);
    return;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rgb_subscriber_node");

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub = node->create_subscription<sensor_msgs::msg::Image>("rgb_image", 5, &rgbCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
