#pragma once

#include <string>
#include <unordered_map>

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "depthai_ros_driver/visibility.h"

namespace depthai_ros_driver {
namespace param_handlers {
class CameraParamHandler : public BaseParamHandler {
   public:
    explicit CameraParamHandler(const std::string& name);
    ~CameraParamHandler(){};
    void declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::ColorCamera> color_cam);
    dai::CameraControl setRuntimeParams(const std::vector<rclcpp::Parameter>& params) override;

   private:
    
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver