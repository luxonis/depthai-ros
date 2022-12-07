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
    virtual ~BaseParamHandler(){};
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
    T declareAndLogParam(rclcpp::Node* node, const std::string& paramName, const std::vector<T>& value) {
        std::string full_name = baseName + "." + paramName;
        if(node->has_parameter(full_name)) {
            return getParam<T>(node, paramName);
        } else {
            logParam(node->get_logger(), full_name, value);
            return node->declare_parameter<T>(full_name, value);
        }
    }

    template <typename T>
    T declareAndLogParam(rclcpp::Node* node, const std::string& paramName, T value) {
        std::string full_name = baseName + "." + paramName;
        if(node->has_parameter(full_name)) {
            return getParam<T>(node, paramName);
        } else {
            logParam(node->get_logger(), full_name, value);
            return node->declare_parameter<T>(full_name, value);
        }
    }
    template <typename T>
    T declareAndLogParam(rclcpp::Node* node, const std::string& paramName, T value, rcl_interfaces::msg::ParameterDescriptor int_range) {
        std::string full_name = baseName + "." + paramName;
        if(node->has_parameter(full_name)) {
            return getParam<T>(node, full_name);
        } else {
            logParam(node->get_logger(), full_name, value);
            return node->declare_parameter<T>(full_name, value, int_range);
        }
    }
    template <typename T>
    inline void logParam(const rclcpp::Logger& logger, const std::string& name, T value) {
        std::stringstream ss;
        ss << value;
        RCLCPP_INFO(logger, "Setting param %s with value %s", name.c_str(), ss.str().c_str());
    }
    template <typename T>
    inline void logParam(const rclcpp::Logger& logger, const std::string& name, const std::vector<T>& value) {
        std::stringstream ss;
        for(const auto& v : value) {
            ss << v << " ";
        }
        RCLCPP_INFO(logger, "Setting param %s with value %s", name.c_str(), ss.str().c_str());
    }
    std::string baseName;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
