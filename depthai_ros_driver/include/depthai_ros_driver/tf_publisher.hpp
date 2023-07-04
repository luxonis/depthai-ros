#pragma once
#include "tf2_ros/static_transform_broadcaster.h"
#include "depthai/device/CalibrationHandler.hpp"
#include "rclcpp/node.hpp"
namespace depthai_ros_driver{
    class TFPublisher{
    public:
        explicit TFPublisher(rclcpp::Node *node, dai::CalibrationHandler handler);
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfPub;

    };
}