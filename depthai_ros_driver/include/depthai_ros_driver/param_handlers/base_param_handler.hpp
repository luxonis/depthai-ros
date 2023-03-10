#pragma once
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
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
    BaseParamHandler(const std::string& name) {
        baseName = name;
    };
    virtual ~BaseParamHandler() = default;
    virtual dai::CameraControl setRuntimeParams(rclcpp::Node* node, const std::vector<rclcpp::Parameter>& params) = 0;
    std::string getName() {
        return baseName;
    }
    template <typename T>
    T getParam(rclcpp::Node* node, const std::string paramName) {
        T value;
        node->get_parameter<T>(baseName + "." + paramName, value);
        return value;
    }
    std::string getFullParamName(const std::string& paramName) {
        return baseName + "." + paramName;
    }

   protected:
    template <typename T>
    T declareAndLogParam(rclcpp::Node* node, const std::string& paramName, const std::vector<T>& value, bool override = false) {
        std::string fullName = baseName + "." + paramName;
        if(node->has_parameter(fullName)) {
            if(override) {
                auto param = rclcpp::Parameter(fullName, value);
                node->set_parameter(param);
            }
            return getParam<T>(node, paramName);
        } else {
            auto val = node->declare_parameter<T>(fullName, value);
            logParam(node->get_logger(), fullName, val);
            return val;
        }
    }

    template <typename T>
    T declareAndLogParam(rclcpp::Node* node, const std::string& paramName, T value, bool override = false) {
        std::string fullName = baseName + "." + paramName;
        if(node->has_parameter(fullName)) {
            if(override) {
                auto param = rclcpp::Parameter(fullName, value);
                node->set_parameter(param);
            }
            return getParam<T>(node, paramName);
        } else {
            auto val = node->declare_parameter<T>(fullName, value);
            logParam(node->get_logger(), fullName, val);
            return val;
        }
    }
    template <typename T>
    T declareAndLogParam(rclcpp::Node* node, const std::string& paramName, T value, rcl_interfaces::msg::ParameterDescriptor int_range, bool override = false) {
        std::string fullName = baseName + "." + paramName;
        if(node->has_parameter(fullName)) {
            if(override) {
                auto param = rclcpp::Parameter(fullName, value);
                node->set_parameter(param);
            }
            return getParam<T>(node, fullName);
        } else {
            auto val = node->declare_parameter<T>(fullName, value, int_range);
            logParam(node->get_logger(), fullName, val);
            return val;
        }
    }
    template <typename T>
    inline void logParam(const rclcpp::Logger& logger, const std::string& name, T value) {
        std::stringstream ss;
        ss << value;
        RCLCPP_DEBUG(logger, "Setting param %s with value %s", name.c_str(), ss.str().c_str());
    }
    template <typename T>
    inline void logParam(const rclcpp::Logger& logger, const std::string& name, const std::vector<T>& value) {
        std::stringstream ss;
        for(const auto& v : value) {
            ss << v << " ";
        }
        RCLCPP_DEBUG(logger, "Setting param %s with value %s", name.c_str(), ss.str().c_str());
    }
    std::string baseName;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
