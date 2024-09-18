
#pragma once

#include <string>
#include <unordered_map>

#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace dai {
enum class UsbSpeed;
}

namespace ros {
class NodeHandle;
class Parameter;
}  // namespace ros

namespace depthai_ros_driver {
namespace param_handlers {

class PipelineGenParamHandler : public BaseParamHandler {
   public:
    explicit PipelineGenParamHandler(ros::NodeHandle node, const std::string& name);
    ~PipelineGenParamHandler();
    void declareParams();
    dai::CameraControl setRuntimeParams(parametersConfig& config) override;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
