#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "depthai-shared/properties/ColorCameraProperties.hpp"
#include "depthai-shared/properties/MonoCameraProperties.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "depthai_ros_driver/parametersConfig.h"

namespace dai {
namespace node {
class MonoCamera;
class ColorCamera;
}  // namespace node
}  // namespace dai

namespace ros {
class NodeHandle;
}  // namespace ros

namespace depthai_ros_driver {
namespace param_handlers {
class SensorParamHandler : public BaseParamHandler {
   public:
    explicit SensorParamHandler(ros::NodeHandle node, const std::string& name);
    ~SensorParamHandler();
    void declareParams(std::shared_ptr<dai::node::MonoCamera> monoCam,
                       dai::CameraBoardSocket socket,
                       dai_nodes::sensor_helpers::ImageSensor sensor,
                       bool publish);
    void declareParams(std::shared_ptr<dai::node::ColorCamera> colorCam,
                       dai::CameraBoardSocket socket,
                       dai_nodes::sensor_helpers::ImageSensor sensor,
                       bool publish);
    dai::CameraControl setRuntimeParams(parametersConfig& config) override;

   private:
    std::unordered_map<std::string, dai::MonoCameraProperties::SensorResolution> monoResolutionMap;
    std::unordered_map<std::string, dai::ColorCameraProperties::SensorResolution> rgbResolutionMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver