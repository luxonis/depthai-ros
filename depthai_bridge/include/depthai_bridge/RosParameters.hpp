#ifndef ROS_PARAMETERS_HPP_
#define ROS_PARAMETERS_HPP_

#include <iostream>
#include <memory>
#include <string>

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/DisparityConverter.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include "depthai/depthai.hpp"

#ifndef IS_ROS2

#include "ros/ros.h"

template <typename T>
static void getParamWithWarning(ros::NodeHandle& pnh, const char* key, T val) {
    bool gotParam = pnh.getParam(key, val);
    if(!gotParam) {
        std::stringstream ss;
        ss << val;
        ROS_WARN("Could not find param '%s' on node '%s'. Defaulting to '%s'", key, pnh.getNamespace().c_str(), ss.str().c_str());
    }
}
#define req_type bool
using ros_node = ros::NodeHandle&;
#define req_get(x) (request.x)
#define rep_get(x) (response.x)
#define set_parameter(a, b) getParamWithWarning(node, a, b)

#else

template <typename T>
static void setRosParameter(std::shared_ptr<rclcpp::Node> node, const char* key, T& val) {
    node->declare_parameter(key, val);
    node->get_parameter(key, val);
}

#define req_type void
using ros_node = std::shared_ptr<rclcpp::Node>;
#define req_get(x) ((*request).x)
#define rep_get(x) ((*response).x)
#define set_parameter(a, b) setRosParameter(node, a, b)
#endif

#endif
