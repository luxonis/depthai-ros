#pragma once

#include <string>
#include <unordered_map>

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
class StereoParamHandler : public BaseParamHandler {
   public:
    explicit StereoParamHandler(const std::string& name);
    ~StereoParamHandler();
    void declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::StereoDepth> stereo);
    dai::CameraControl setRuntimeParams(rclcpp::Node* node, const std::vector<rclcpp::Parameter>& params) override;

   private:
    std::unordered_map<std::string, dai::node::StereoDepth::PresetMode> depthPresetMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver