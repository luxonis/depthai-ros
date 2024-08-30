#pragma once
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/parametersConfig.h"
#include "ros/node_handle.h"

namespace depthai_ros_driver {
namespace param_handlers {
inline std::pair<int, int> getRangedIntDescriptor(uint16_t min, uint16_t max) {
    return std::make_pair<int, int>(min, max);
}

class BaseParamHandler {
   public:
    BaseParamHandler(ros::NodeHandle node, const std::string& name) : baseName(name), baseNode(node){};
    virtual ~BaseParamHandler(){};
    virtual dai::CameraControl setRuntimeParams(parametersConfig& config) = 0;
    std::string getName() {
        return baseName;
    }
    template <typename T>
    T getParam(const std::string& paramName) {
        T value;
        if(!baseNode.hasParam(getFullParamName(paramName))) {
            ROS_ERROR("Param %s not found", getFullParamName(paramName).c_str());
        }
        baseNode.getParam(getFullParamName(paramName), value);
        logParam(getFullParamName(paramName), value);
        return value;
    }
    template <typename T>
    T getParam(const std::string& paramName, T defaultVal) {
        T value;
        if(!baseNode.hasParam(getFullParamName(paramName))) {
            ROS_ERROR("Param %s not found", getFullParamName(paramName).c_str());
        }
        if(!baseNode.param<T>(getFullParamName(paramName), value, defaultVal)) {
            baseNode.setParam(getFullParamName(paramName), defaultVal);
            value = defaultVal;
        }
        logParam(getFullParamName(paramName), value);
        return value;
    }
    template <typename T>
    T getOtherNodeParam(const std::string& daiNodeName, const std::string& paramName) {
        T value;
        baseNode.getParam(getFullParamName(daiNodeName, paramName), value);
        return value;
    }
    template <typename T>
    T getOtherNodeParam(const std::string& daiNodeName, const std::string& paramName, T defaultVal) {
        T value;
        if(!baseNode.param<T>(getFullParamName(daiNodeName, paramName), value, defaultVal)) {
            baseNode.setParam(getFullParamName(daiNodeName, paramName), defaultVal);
            value = defaultVal;
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
        std::string name = baseName + "_" + paramName;
        return name;
    }
    std::string getFullParamName(const std::string& daiNodeName, const std::string& paramName) {
        std::string name = daiNodeName + "_" + paramName;
        return name;
    }
    std::string getSocketName(dai::CameraBoardSocket socket) {
        return dai_nodes::sensor_helpers::getSocketName(getROSNode(), socket);
    }

   protected:
    ros::NodeHandle getROSNode() {
        return baseNode;
    }

    template <typename T>
    T declareAndLogParam(const std::string& paramName, const std::vector<T>& value, bool override = false) {
        std::string fullName = getFullParamName(paramName);
        if(override || !baseNode.hasParam(fullName)) {
            return setParam(paramName, value);
        } else {
            return getParam<T>(paramName);
        }
    }

    template <typename T>
    T declareAndLogParam(const std::string& paramName, T value, bool override = false) {
        std::string fullName = getFullParamName(paramName);
        if(override || !baseNode.hasParam(fullName)) {
            return setParam(paramName, value);
        } else {
            return getParam<T>(paramName);
        }
    }
    int declareAndLogParam(const std::string& paramName, int value, std::pair<int, int> int_range, bool override = false) {
        std::string fullName = getFullParamName(paramName);
        if(override || !baseNode.hasParam(fullName)) {
            if(value < int_range.first || value > int_range.second) {
                ROS_WARN("Param %s with value %d is out of range [%d, %d]. Setting value %d",
                         fullName.c_str(),
                         value,
                         int_range.first,
                         int_range.second,
                         int_range.first);
                value = int_range.first;
            }
            return setParam(paramName, value);
        } else {
            return getParam<int>(paramName);
        }
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
