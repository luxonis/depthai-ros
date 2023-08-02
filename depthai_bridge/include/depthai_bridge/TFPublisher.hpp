#pragma once
#include "tf2_ros/static_transform_broadcaster.h"
#include "depthai/device/CalibrationHandler.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter_client.hpp"

namespace dai {

namespace ros {
    class TFPublisher{
    public:
        explicit TFPublisher(rclcpp::Node *node, dai::CalibrationHandler handler);
        std::string getXacro();
    private:
        std::unique_ptr<rclcpp::AsyncParametersClient> paramClient;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfPub;
    };
}
}