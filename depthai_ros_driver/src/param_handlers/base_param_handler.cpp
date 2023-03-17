#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

#include <string>

#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
BaseParamHandler::BaseParamHandler(const std::string& name) {
    baseName = name;
};
std::string BaseParamHandler::getName() {
    return baseName;
}

template <typename T>
T BaseParamHandler::getParam(rclcpp::Node* node, const std::string paramName) {
    T value;
    node->get_parameter<T>(baseName + "." + paramName, value);
    return value;
}

// template <typename T>
// T BaseParamHandler::declareAndLogParam(rclcpp::Node* node, const std::string& paramName, const std::vector<T>& value, bool override) {
//     std::string fullName = baseName + "." + paramName;
//     if(node->has_parameter(fullName)) {
//         if(override) {
//             auto param = rclcpp::Parameter(fullName, value);
//             node->set_parameter(param);
//         }
//         return getParam<T>(node, paramName);
//     } else {
//         auto val = node->declare_parameter<T>(fullName, value);
//         logParam(node->get_logger(), fullName, val);
//         return val;
//     }
// }

template <typename T>
T BaseParamHandler::declareAndLogParam(rclcpp::Node* node, const std::string& paramName, T value, bool override) {
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
T BaseParamHandler::declareAndLogParam(
    rclcpp::Node* node, const std::string& paramName, T value, rcl_interfaces::msg::ParameterDescriptor int_range, bool override) {
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
void BaseParamHandler::logParam(const rclcpp::Logger& logger, const std::string& name, T value) {
    std::stringstream ss;
    ss << value;
    RCLCPP_DEBUG(logger, "Setting param %s with value %s", name.c_str(), ss.str().c_str());
}
template <>
int BaseParamHandler::getParam<int>(rclcpp::Node* node, const std::string paramName);
template <>
double BaseParamHandler::getParam<double>(rclcpp::Node* node, const std::string paramName);
template <>
bool BaseParamHandler::getParam<bool>(rclcpp::Node* node, const std::string paramName);
template <>
std::string BaseParamHandler::getParam<std::string>(rclcpp::Node* node, const std::string paramName);

// extern template int BaseParamHandler::declareAndLogParam<int>(rclcpp::Node* node, const std::string& paramName, const std::vector<int>& value, bool
// override); extern template double BaseParamHandler::declareAndLogParam<double>(rclcpp::Node* node,
//                                                                     const std::string& paramName,
//                                                                     const std::vector<double>& value,
//                                                                     bool override);
// extern template bool BaseParamHandler::declareAndLogParam<bool>(rclcpp::Node* node,
//                                                                 const std::string& paramName,
//                                                                 const std::vector<bool>& value,
//                                                                 bool override);
// extern template std::string BaseParamHandler::declareAndLogParam<std::string>(rclcpp::Node* node,
//                                                                               const std::string& paramName,
//                                                                               const std::vector<std::string>& value,
//                                                                               bool override);
// extern template std::basic_string<char, std::char_traits<char>, std::allocator<char>>
// BaseParamHandler::declareAndLogParam<std::basic_string<char, std::char_traits<char>, std::allocator<char>>>(
//     rclcpp::Node* node,
//     const std::string& paramName,
//     const std::vector<std::basic_string<char, std::char_traits<char>, std::allocator<char>>>& value,
// bool override);

template <>
int BaseParamHandler::declareAndLogParam<int>(rclcpp::Node* node, const std::string& paramName, int value, bool override);
template <>
double BaseParamHandler::declareAndLogParam<double>(rclcpp::Node* node, const std::string& paramName, double value, bool override);
template <>
bool BaseParamHandler::declareAndLogParam<bool>(rclcpp::Node* node, const std::string& paramName, bool value, bool override);
template <>
std::string BaseParamHandler::declareAndLogParam<std::string>(rclcpp::Node* node, const std::string& paramName, std::string value, bool override);

template int BaseParamHandler::declareAndLogParam<int>(
    rclcpp::Node* node, const std::string& paramName, int value, rcl_interfaces::msg::ParameterDescriptor int_range, bool override);

extern template void BaseParamHandler::logParam<int>(const rclcpp::Logger& logger, const std::string& name, int value);
extern template void BaseParamHandler::logParam<double>(const rclcpp::Logger& logger, const std::string& name, double value);
extern template void BaseParamHandler::logParam<bool>(const rclcpp::Logger& logger, const std::string& name, bool value);
extern template void BaseParamHandler::logParam<std::string>(const rclcpp::Logger& logger, const std::string& name, std::string value);

}  // namespace param_handlers
}  // namespace depthai_ros_driver