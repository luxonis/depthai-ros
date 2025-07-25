#pragma once
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "rclcpp/node.hpp"
namespace depthai_ros_driver {
namespace param_handlers {
inline rcl_interfaces::msg::ParameterDescriptor getRangedIntDescriptor(uint16_t min, uint16_t max) {
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.integer_range.resize(1);
        desc.integer_range.at(0).from_value = min;
        desc.integer_range.at(0).to_value = max;
        return desc;
    }
}
class BaseParamHandler {
   public:
    BaseParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name,const std::string& deviceName, bool rsCompat)
        : baseName(name), deviceName(deviceName), rsCompat(rsCompat), baseNode(node) {};
    virtual ~BaseParamHandler() = default;
    virtual std::shared_ptr<dai::CameraControl> setRuntimeParams(const std::vector<rclcpp::Parameter>& /* params */) {
        return std::make_shared<dai::CameraControl>();
    }
    std::string getName() {
        return baseName;
    }
    template <typename T>
    T getParam(const std::string& paramName) {
        T value;
        if(!baseNode->has_parameter(getFullParamName(paramName))) {
            RCLCPP_WARN(baseNode->get_logger(), "Parameter %s not found", getFullParamName(paramName).c_str());
        }
        baseNode->get_parameter<T>(getFullParamName(paramName), value);
        return value;
    }
    template <typename T>
    T getOtherNodeParam(const std::string& daiNodeName, const std::string& paramName) {
        T value;
        if(!baseNode->has_parameter(getFullParamName(daiNodeName, paramName))) {
            RCLCPP_WARN(baseNode->get_logger(), "Parameter %s not found", getFullParamName(daiNodeName, paramName).c_str());
        }
        baseNode->get_parameter<T>(getFullParamName(daiNodeName, paramName), value);
        return value;
    }

    std::string getFullParamName(const std::string& paramName) {
        return baseName + "." + paramName;
    }
    std::string getFullParamName(const std::string& daiNodeName, const std::string& paramName) {
        std::string name = daiNodeName + "." + paramName;
        return name;
    }

   protected:
    std::shared_ptr<rclcpp::Node> getROSNode() {
        return baseNode;
    }
    std::string getSocketName(dai::CameraBoardSocket socket) {
        return depthai_bridge::getSocketName(socket, deviceName, rsCompat);
    }
    template <typename T>
    T declareAndLogParam(const std::string& paramName, const std::vector<T>& value, bool override = false) {
        std::string fullName = baseName + "." + paramName;
        if(baseNode->has_parameter(fullName)) {
            if(override) {
                auto param = rclcpp::Parameter(fullName, value);
                baseNode->set_parameter(param);
            }
            return getParam<T>(paramName);
        } else {
            auto val = baseNode->declare_parameter<T>(fullName, value);
            logParam(fullName, val);
            return val;
        }
    }

    template <typename T>
    T declareAndLogParam(const std::string& paramName, T value, bool override = false) {
        std::string fullName = baseName + "." + paramName;
        if(baseNode->has_parameter(fullName)) {
            if(override) {
                auto param = rclcpp::Parameter(fullName, value);
                baseNode->set_parameter(param);
            }
            return getParam<T>(paramName);
        } else {
            auto val = baseNode->declare_parameter<T>(fullName, value);
            logParam(fullName, val);
            return val;
        }
    }
    template <typename T>
    T declareAndLogParam(const std::string& paramName, T value, rcl_interfaces::msg::ParameterDescriptor int_range, bool override = false) {
        std::string fullName = baseName + "." + paramName;
        if(baseNode->has_parameter(fullName)) {
            if(override) {
                auto param = rclcpp::Parameter(fullName, value);
                baseNode->set_parameter(param);
            }
            return getParam<T>(fullName);
        } else {
            auto val = baseNode->declare_parameter<T>(fullName, value, int_range);
            logParam(fullName, val);
            return val;
        }
    }
    template <typename T>
    inline void logParam(const std::string& name, T value) {
        std::stringstream ss;
        ss << value;
        RCLCPP_DEBUG(baseNode->get_logger(), "Setting param %s with value %s", name.c_str(), ss.str().c_str());
    }
    template <typename T>
    inline void logParam(const std::string& name, const std::vector<T>& value) {
        std::stringstream ss;
        for(const auto& v : value) {
            ss << v << " ";
        }
        RCLCPP_DEBUG(baseNode->get_logger(), "Setting param %s with value %s", name.c_str(), ss.str().c_str());
    }
    std::string baseName;
    std::string deviceName;
    bool rsCompat;
    std::shared_ptr<rclcpp::Node> baseNode;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
