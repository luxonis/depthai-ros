#pragma once

#include <string>
#include <unordered_map>

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
class MonoParamHandler : public BaseParamHandler {
   public:
    explicit MonoParamHandler(const std::string& name);
    ~MonoParamHandler();
    void declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::MonoCamera> mono_cam, dai::CameraBoardSocket socket, dai_nodes::sensor_helpers::ImageSensor sensor);
    dai::CameraControl setRuntimeParams(rclcpp::Node* node, const std::vector<rclcpp::Parameter>& params) override;

   private:
    std::unordered_map<std::string, dai::MonoCameraProperties::SensorResolution> monoResolutionMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver