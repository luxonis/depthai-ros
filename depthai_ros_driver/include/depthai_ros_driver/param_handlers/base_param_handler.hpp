#pragma once
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
namespace depthai_ros_driver {
namespace param_handlers {
class BaseParamHandler {
   public:
    BaseParamHandler(const std::string &dai_node_name){
        dai_node_name_ = dai_node_name;
    };
    virtual void declareParams(rclcpp::Node* node, std::shared_ptr<dai::Node> dai_node);
    virtual dai::CameraControl setRuntimeParams(const std::vector<rclcpp::Parameter>& params);

   protected:
    rcl_interfaces::msg::ParameterDescriptor get_ranged_int_descriptor(uint16_t min, uint16_t max) {
        {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.integer_range.resize(1);
            desc.integer_range.at(0).from_value = min;
            desc.integer_range.at(0).to_value = max;
            return desc;
        }
    }
    template <typename T>
    T declareAndLogParam(rclcpp::Node *node, const std::string &param_name, T value){
        std::string full_name = dai_node_name_ + "." + param_name;
        
        auto p = rclcpp::Parameter(full_name, value);
        log_param(node->get_logger(), full_name, p.as_string());
        return node->declare_parameter(full_name, value);
    }
    template <typename T>
    T declareAndLogParam(rclcpp::Node *node, const std::string &param_name, T value, rcl_interfaces::msg::ParameterDescriptor int_range){
        std::string full_name = dai_node_name_ + "." + param_name;
        
        auto p = rclcpp::Parameter(full_name, value);
        log_param(node->get_logger(), full_name, p.as_string());
        return node->declare_parameter(full_name, value, int_range);
    }
    inline void log_param(const rclcpp::Logger& logger, const std::string & name,std::string &value){
        RCLCPP_INFO(logger, "Setting param %s with value %s", name, value);
    };
    std::string dai_node_name_;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
