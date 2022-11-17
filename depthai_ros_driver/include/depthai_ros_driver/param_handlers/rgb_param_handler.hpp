#pragma once

#include <string>
#include <unordered_map>

#include "depthai/include/depthai-shared/properties/ColorCameraProperties.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
class RGBParamHandler : public BaseParamHandler {
   public:
    RGBParamHandler(const std::string &dai_node_name);
    void declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::ColorCamera> color_cam);
    dai::CameraControl setRuntimeParams(const std::vector<rclcpp::Parameter>& params) override;
   private:
    std::unordered_map<std::string, dai::ColorCameraProperties::SensorResolution> rgb_resolution_map_ = {
        {"1080", dai::ColorCameraProperties::SensorResolution::THE_1080_P},
        {"4k", dai::ColorCameraProperties::SensorResolution::THE_4_K},
        {"12MP", dai::ColorCameraProperties::SensorResolution::THE_12_MP},
    };
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver