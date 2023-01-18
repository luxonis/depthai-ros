#pragma once

#include <string>
#include <unordered_map>

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
class RGBParamHandler : public BaseParamHandler {
   public:
    explicit RGBParamHandler(const std::string& name);
    ~RGBParamHandler();
    void declareParams(ros::NodeHandle node,
                       std::shared_ptr<dai::node::ColorCamera> colorCam,
                       dai::CameraBoardSocket socket,
                       dai_nodes::sensor_helpers::ImageSensor sensor,
                       bool publish);
    dai::CameraControl setRuntimeParams(ros::NodeHandle node, parametersConfig& config) override;

   private:
    std::unordered_map<std::string, dai::ColorCameraProperties::SensorResolution> rgbResolutionMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver