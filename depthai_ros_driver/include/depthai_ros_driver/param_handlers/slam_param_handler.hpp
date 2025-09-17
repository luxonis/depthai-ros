#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace dai {
namespace node {
class RTABMapSLAM;
}
}  // namespace dai

namespace rclcpp {
class Node;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
class SlamParamHandler : public BaseParamHandler {
   public:
    explicit SlamParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat);
    ~SlamParamHandler();
    void declareParams(std::shared_ptr<dai::node::RTABMapSLAM> slam);
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
