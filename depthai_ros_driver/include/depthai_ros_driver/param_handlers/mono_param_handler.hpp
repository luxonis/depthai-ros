#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "depthai-shared/properties/MonoCameraProperties.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace dai {
namespace node {
class MonoCamera;
}
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
class MonoParamHandler : public BaseParamHandler {
   public:
    explicit MonoParamHandler(rclcpp::Node* node, const std::string& name);
    ~MonoParamHandler();
    void declareParams(std::shared_ptr<dai::node::MonoCamera> monoCam,
                       dai::CameraBoardSocket socket,
                       dai_nodes::sensor_helpers::ImageSensor sensor,
                       bool publish);
    dai::CameraControl setRuntimeParams(const std::vector<rclcpp::Parameter>& params) override;

   private:
    std::unordered_map<std::string, dai::MonoCameraProperties::SensorResolution> monoResolutionMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver