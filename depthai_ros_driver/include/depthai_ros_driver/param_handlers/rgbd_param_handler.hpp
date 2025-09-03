#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace dai {
namespace node {
class RGBD;
}
}  // namespace dai

namespace rclcpp {
class Node;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
class RGBDParamHandler : public BaseParamHandler {
   public:
    explicit RGBDParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat);
    ~RGBDParamHandler();
    void declareParams(std::shared_ptr<dai::node::RGBD> rgbd, dai::CameraBoardSocket socket);
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
