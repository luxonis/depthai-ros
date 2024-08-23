#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai/pipeline/datatype/CameraControl.hpp"
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
    explicit ToFParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name);
    ~ToFParamHandler();
    void declareParams(std::shared_ptr<dai::node::Camera> cam, std::shared_ptr<dai::node::ToF> tof);
    dai::CameraControl setRuntimeParams(const std::vector<rclcpp::Parameter>& params) override;
    std::unordered_map<std::string, dai::MedianFilter> medianFilterMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
