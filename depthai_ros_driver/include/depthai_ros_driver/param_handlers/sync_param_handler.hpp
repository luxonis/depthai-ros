
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace dai {
namespace node {
class Sync;
}
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
class SyncParamHandler : public BaseParamHandler {
   public:
    explicit SyncParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name);
    ~SyncParamHandler();
    void declareParams(std::shared_ptr<dai::node::Sync> sync);
    dai::CameraControl setRuntimeParams(const std::vector<rclcpp::Parameter>& params) override;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
