#pragma once

#include <string>
#include <unordered_map>

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
namespace camera {
enum class NNType { None, RGB, Spatial };
}
class CameraParamHandler : public BaseParamHandler {
   public:
    explicit CameraParamHandler(const std::string& name);
    ~CameraParamHandler();
    void declareParams(ros::NodeHandle node);
    dai::CameraControl setRuntimeParams(ros::NodeHandle node, parametersConfig& config) override;
    camera::NNType getNNType(ros::NodeHandle node);
    dai::UsbSpeed getUSBSpeed(ros::NodeHandle node);

   private:
    std::unordered_map<std::string, dai::UsbSpeed> usbSpeedMap;
    std::unordered_map<std::string, camera::NNType> nnTypeMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver