#pragma once

#include <string>
#include <unordered_map>

#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace dai {
enum class UsbSpeed;
}

namespace ros {
class NodeHandle;
}  // namespace ros

namespace depthai_ros_driver {
namespace param_handlers {
class CameraParamHandler : public BaseParamHandler {
   public:
    explicit CameraParamHandler(ros::NodeHandle node, const std::string& name);
    ~CameraParamHandler();
    void declareParams();
    dai::CameraControl setRuntimeParams(parametersConfig& config) override;
    dai::UsbSpeed getUSBSpeed();

   private:
    std::unordered_map<std::string, dai::UsbSpeed> usbSpeedMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver