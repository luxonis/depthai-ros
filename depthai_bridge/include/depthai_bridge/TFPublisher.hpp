#pragma once
#include "tf2_ros/static_transform_broadcaster.h"
#include "depthai/device/CalibrationHandler.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter_client.hpp"

namespace dai {

namespace ros {
    class TFPublisher{
    public:
        explicit TFPublisher(rclcpp::Node *node, dai::CalibrationHandler handler, const std::string& xacroArgs);
        std::string getXacro(const std::string& xacroArgs);
        geometry_msgs::msg::Quaternion quatFromRotM(nlohmann::json rotMatrix);
        geometry_msgs::msg::Vector3 transFromExtr(nlohmann::json translation);
    private:
        std::unique_ptr<rclcpp::AsyncParametersClient> paramClient;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfPub;
    };
}
}