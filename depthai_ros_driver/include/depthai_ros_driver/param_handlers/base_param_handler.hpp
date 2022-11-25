#pragma once
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
namespace depthai_ros_driver {
namespace param_handlers {
inline rcl_interfaces::msg::ParameterDescriptor get_ranged_int_descriptor(uint16_t min, uint16_t max) {
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
        name_ = name;
    };
    virtual ~BaseParamHandler(){};
    virtual dai::CameraControl setRuntimeParams(rclcpp::Node* node, const std::vector<rclcpp::Parameter>& params) = 0;
    template <typename T>
    T get_param(rclcpp::Node* node, const std::string param_name) {
        T value;
        node->get_parameter<T>(name_ + "." + param_name, value);
        return value;
    }
    std::string get_full_param_name(const std::string& param_name) {
        return name_ + "." + param_name;
    }

   protected:
    template <typename T>
    T declareAndLogParam(rclcpp::Node* node, const std::string& param_name, T value) {
        std::string full_name = name_ + "." + param_name;

        log_param(node->get_logger(), full_name, value);

        return node->declare_parameter<T>(full_name, value);
    }
    template <typename T>
    T declareAndLogParam(rclcpp::Node* node, const std::string& param_name, T value, rcl_interfaces::msg::ParameterDescriptor int_range) {
        std::string full_name = name_ + "." + param_name;

        log_param(node->get_logger(), full_name, value);
        return node->declare_parameter(full_name, value, int_range);
    }
    template <typename T>
    inline void log_param(const rclcpp::Logger& logger, const std::string& name, T value) {
        std::stringstream ss;
        ss << value;
        RCLCPP_INFO(logger, "Setting param %s with value %s", name.c_str(), ss.str().c_str());
    }
    std::string name_;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
