#pragma once
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "ros/ros.h"
#include "depthai_ros_driver/parametersConfig.h"

namespace depthai_ros_driver {
namespace param_handlers {

class BaseParamHandler {
   public:
    BaseParamHandler(const std::string& name) {
        baseName = name;
    };
    virtual ~BaseParamHandler(){};
    virtual dai::CameraControl setRuntimeParams(ros::NodeHandle node, parametersConfig &config) = 0;
    std::string getName() {
        return baseName;
    }
    template <typename T>
    T getParam(ros::NodeHandle node, const std::string paramName) {
        T value;
        node.getParam(getFullParamName(node, paramName), value);
        logParam(getFullParamName(node, paramName),value);
        return value;
    }
    std::string getFullParamName(ros::NodeHandle node, const std::string& paramName) {
        return std::string(node.getNamespace()) +"/" + baseName + "_" + paramName;
    }
    std::string baseName;
    private:
    template <typename T>
    inline void logParam(const std::string& name, T value) {
        std::stringstream ss;
        ss << value;
        ROS_INFO("Param %s with value %s", name.c_str(), ss.str().c_str());
    }
    template <typename T>
    inline void logParam(const std::string& name, const std::vector<T>& value) {
        std::stringstream ss;
        for(const auto& v : value) {
            ss << v << " ";
        }
        ROS_INFO("Param %s with value %s", name.c_str(), ss.str().c_str());
    }
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
