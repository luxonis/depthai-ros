#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace dai {
namespace node {
class MonoCamera;
class ColorCamera;
}  // namespace node
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
class SensorParamHandler : public BaseParamHandler {
   public:
    explicit SensorParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, dai::CameraBoardSocket socket);
    ~SensorParamHandler();
    void declareCommonParams(dai::CameraBoardSocket socket);
    void declareParams(std::shared_ptr<dai::node::MonoCamera> monoCam, dai_nodes::sensor_helpers::ImageSensor sensor, bool publish);
    void declareParams(std::shared_ptr<dai::node::ColorCamera> colorCam, dai_nodes::sensor_helpers::ImageSensor sensor, bool publish);
    dai::CameraControl setRuntimeParams(const std::vector<rclcpp::Parameter>& params) override;

   private:
    dai::CameraBoardSocket socketID;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
