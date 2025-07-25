#pragma once

#include <memory>
#include <string>

#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace dai {
namespace node {
class Camera;
class ToF;
}  // namespace node
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {

class ToFParamHandler : public BaseParamHandler {
   public:
    explicit ToFParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat);
    ~ToFParamHandler();
    void declareParams(std::shared_ptr<dai::node::ToF> tof);
    std::unordered_map<std::string, dai::MedianFilter> medianFilterMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
