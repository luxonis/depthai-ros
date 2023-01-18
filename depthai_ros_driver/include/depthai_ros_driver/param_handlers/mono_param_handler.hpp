#pragma once

#include <string>
#include <unordered_map>

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
class MonoParamHandler : public BaseParamHandler {
   public:
    explicit MonoParamHandler(const std::string& name);
    ~MonoParamHandler();
    void declareParams(ros::NodeHandle node,
                       std::shared_ptr<dai::node::MonoCamera> monoCam,
                       dai::CameraBoardSocket socket,
                       dai_nodes::sensor_helpers::ImageSensor sensor,
                       bool publish);
    dai::CameraControl setRuntimeParams(ros::NodeHandle node, parametersConfig& config) override;

   private:
    std::unordered_map<std::string, dai::MonoCameraProperties::SensorResolution> monoResolutionMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver