#pragma once
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
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
    BaseParamHandler(rclcpp::Node* node, const std::string& name) {
        baseName = name;
        baseNode = node;
    };
    virtual ~BaseParamHandler() = default;
    virtual dai::CameraControl setRuntimeParams(const std::vector<rclcpp::Parameter>& params) = 0;
    std::string getName() {
        return baseName;
    }
    template <typename T>
    T getParam(const std::string paramName) {
        T value;
        baseNode->get_parameter<T>(getFullParamName(paramName), value);
        return value;
    }
    template <typename T>
    T getOtherNodeParam(const std::string& daiNodeName, const std::string& paramName) {
        T value;
        baseNode->get_parameter<T>(getFullParamName(daiNodeName, paramName), value);
        return value;
    }

    std::string getFullParamName(const std::string& paramName) {
        return baseName + "." + paramName;
    }
    std::string getFullParamName(const std::string& daiNodeName, const std::string& paramName) {
        std::string name = std::string(baseNode->get_namespace()) + "/" + daiNodeName + "." + paramName;
        return name;
    }

   protected:
    rclcpp::Node* getROSNode() {
        return baseNode;
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
    rclcpp::Node* baseNode;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
