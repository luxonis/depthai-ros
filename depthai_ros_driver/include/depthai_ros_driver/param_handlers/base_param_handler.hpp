#pragma once
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai_ros_driver/parametersConfig.h"
#include "ros/node_handle.h"

namespace depthai_ros_driver {
namespace param_handlers {

class BaseParamHandler {
   public:
    BaseParamHandler(ros::NodeHandle node, const std::string& name) {
        baseName = name;
        baseNode = node;
    };
    virtual ~BaseParamHandler(){};
    virtual dai::CameraControl setRuntimeParams(parametersConfig& config) = 0;
    std::string getName() {
        return baseName;
    }
    template <typename T>
    T getParam(const std::string& paramName) {
        T value;
        baseNode.getParam(getFullParamName(paramName), value);
        return value;
    }
    template <typename T>
    T getParam(const std::string& paramName, T defaultVal) {
        T value;
        if(!baseNode.param<T>(getFullParamName(paramName), value, defaultVal)) {
            baseNode.setParam(getFullParamName(paramName), defaultVal);
        }
        return value;
    }
    template <typename T>
    T setParam(const std::string& paramName, T value) {
        logParam(getFullParamName(paramName), value);
        baseNode.setParam(getFullParamName(paramName), value);
        return value;
    }
    std::string getFullParamName(const std::string& paramName) {
        return std::string(baseNode.getNamespace()) + "/" + baseName + "_" + paramName;
    }

   protected:
    ros::NodeHandle getROSNode() {
        return baseNode;
    }
    template <typename T>
    inline void logParam(const std::string& name, T value) {
        std::stringstream ss;
        ss << value;
        ROS_DEBUG("Param %s with value %s", name.c_str(), ss.str().c_str());
    }
    template <typename T>
    inline void logParam(const std::string& name, const std::vector<T>& value) {
        std::stringstream ss;
        for(const auto& v : value) {
            ss << v << " ";
        }
        ROS_DEBUG("Param %s with value %s", name.c_str(), ss.str().c_str());
    }
    std::string baseName;
    ros::NodeHandle baseNode;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
