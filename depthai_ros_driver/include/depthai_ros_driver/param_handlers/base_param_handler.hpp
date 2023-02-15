#pragma once
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai_ros_driver/parametersConfig.h"
#include "ros/ros.h"

namespace depthai_ros_driver {
namespace param_handlers {

class BaseParamHandler {
   public:
    BaseParamHandler(const std::string& name) {
        baseName = name;
    };
    virtual ~BaseParamHandler(){};
    virtual dai::CameraControl setRuntimeParams(ros::NodeHandle node, parametersConfig& config) = 0;
    std::string getName() {
        return baseName;
    }
    template <typename T>
    T getParam(ros::NodeHandle node, const std::string& paramName) {
        T value;
        node.getParam(getFullParamName(node, paramName), value);
        return value;
    }
    template <typename T>
    T getParam(ros::NodeHandle node, const std::string& paramName, T defaultVal) {
        T value;
        if(!node.param<T>(getFullParamName(node, paramName), value, defaultVal)) {
            node.setParam(getFullParamName(node, paramName), defaultVal);
        }
        return value;
    }
    template <typename T>
    T setParam(ros::NodeHandle node, const std::string& paramName, T value) {
        logParam(getFullParamName(node, paramName), value);
        node.setParam(getFullParamName(node, paramName), value);
        return value;
    }
    std::string getFullParamName(ros::NodeHandle node, const std::string& paramName) {
        return std::string(node.getNamespace()) + "/" + baseName + "_" + paramName;
    }
    std::string baseName;

   private:
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
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
