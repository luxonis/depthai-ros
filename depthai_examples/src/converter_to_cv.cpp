#include <random>

#include "cv_bridge/cv_bridge.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"

cv::Mat generateRandomImage(int width, int height) {
    cv::Mat randomImage(height, width, CV_8UC3);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);

    for(int i = 0; i < height; ++i) {
        for(int j = 0; j < width; ++j) {
            randomImage.at<cv::Vec3b>(i, j) = cv::Vec3b(dis(gen), dis(gen), dis(gen));
        }
    }
    return randomImage;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto inputConverter = std::make_shared<depthai_bridge::ImageConverter>("rgb_frame", true);

    auto node = rclcpp::Node::make_shared("oak");

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub =
        node->create_subscription<sensor_msgs::msg::Image>("rgb_image", 5, [&](const sensor_msgs::msg::Image::SharedPtr rgbImageMsg) {
            cv::Mat rgbImage = inputConverter->rosMsgtoCvMat(*rgbImageMsg);
            cv::imshow("video", rgbImage);
            cv::waitKey(1);
            return;
        });

    auto random_image_publisher = node->create_publisher<sensor_msgs::msg::Image>("rgb_image", 5);

    auto timer_callback = [&]() {
        cv::Mat randomImage = generateRandomImage(640, 480);

        sensor_msgs::msg::Image::SharedPtr randomImageMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", randomImage).toImageMsg();
        random_image_publisher->publish(*randomImageMsg);
    };

    rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(std::chrono::seconds(1), timer_callback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
