#include <ctype.h>
#include <stdio.h>

#include <chrono>

#include "depthai_ros_msgs/srv/normalized_image_crop.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

static constexpr float stepSize = 0.02;

void boundAdjuster(double& value) {
    if(value < 0) {
        std::cout << "values should always be greater than 0." << std::endl;
        std::cout << "Resetting to 0." << std::endl;
        value = 0;
    } else if(value > 1) {
        std::cout << "values should always be less than 1." << std::endl;
        std::cout << "Resetting to 1." << std::endl;
        value = 1;
    }
    return;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("crop_control_service");
    std::string serviceName;
    node->declare_parameter("service_name", "crop_control_srv");
    node->get_parameter("service_name", serviceName);

    rclcpp::Client<depthai_ros_msgs::srv::NormalizedImageCrop>::SharedPtr client = node->create_client<depthai_ros_msgs::srv::NormalizedImageCrop>(serviceName);

    auto request = std::make_shared<depthai_ros_msgs::srv::NormalizedImageCrop::Request>();
    request->top_left.x = 0.2;
    request->top_left.y = 0.2;
    request->bottom_right.x = 0.2;
    request->bottom_right.y = 0.2;

    while(!client->wait_for_service(1s)) {
        if(!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    std::cout << "Use the following keys to control the cropping region" << std::endl;
    std::cout << "  Q/W -> Increment/Decrement the topleft X position" << std::endl;
    std::cout << "  A/S -> Increment/Decrement the topleft Y position" << std::endl;
    std::cout << "  E/R -> Increment/Decrement the bottomright X position" << std::endl;
    std::cout << "  D/F -> Increment/Decrement the bottomright Y position" << std::endl;
    std::cout << "  Preess ctrl+D to exit." << std::endl;
    char c;
    bool sendSignal = false;

    while(rclcpp::ok()) {
        c = std::tolower(getchar());
        switch(c) {
            case 'w':
                request->top_left.x -= stepSize;
                boundAdjuster(request->top_left.x);
                sendSignal = true;
                break;
            case 'q':
                request->top_left.x += stepSize;
                boundAdjuster(request->top_left.x);
                sendSignal = true;
                break;
            case 'a':
                request->top_left.y += stepSize;
                boundAdjuster(request->top_left.y);
                sendSignal = true;
                break;
            case 's':
                request->top_left.y -= stepSize;
                boundAdjuster(request->top_left.y);
                sendSignal = true;
                break;
            case 'e':
                request->bottom_right.x += stepSize;
                boundAdjuster(request->bottom_right.x);
                sendSignal = true;
                break;
            case 'r':
                request->bottom_right.x -= stepSize;
                boundAdjuster(request->bottom_right.x);
                sendSignal = true;
                break;
            case 'd':
                request->bottom_right.y += stepSize;
                boundAdjuster(request->bottom_right.y);
                sendSignal = true;
                break;
            case 'f':
                request->bottom_right.y -= stepSize;
                boundAdjuster(request->bottom_right.y);
                sendSignal = true;
                break;
            default:
                // TODO(sachin): Use RCLCPP_INFO instead of cout.
                std::cout << " Entered Invalid Key..!!!" << std::endl;
                std::cout << " Use the following keys to control the cropping region" << std::endl;
                std::cout << "  Q/W -> Increment/Decrement the topleft X position" << std::endl;
                std::cout << "  A/S -> Increment/Decrement the topleft Y position" << std::endl;
                std::cout << "  E/R -> Increment/Decrement the bottomright X position" << std::endl;
                std::cout << "  D/F -> Increment/Decrement the bottomright Y position" << std::endl;
                std::cout << "  Preess ctrl+D to exit." << std::endl;
        }

        if(sendSignal) {
            std::cout << "Top left Position -> (" << request->top_left.x << ", " << request->top_left.y << ")" << std::endl;
            std::cout << "Bottion right Position -> (" << request->bottom_right.x << ", " << request->bottom_right.y << ")" << std::endl;
            auto result = client->async_send_request(request);
            if(rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Status: %ld", result.get()->status);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service crop_control_srv");
            }
            sendSignal = false;
        }
    }

    return 0;
}