#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "depthai_ros_driver/parametersConfig.h"

namespace dai {
namespace node {
class IMU;
}
}  // namespace dai

namespace ros {
class NodeHandle;
}  // namespace ros

namespace depthai_ros_driver {
namespace param_handlers {
class ImuParamHandler : public BaseParamHandler {
   public:
    explicit ImuParamHandler(ros::NodeHandle node, const std::string& name);
    ~ImuParamHandler();
    void declareParams(std::shared_ptr<dai::node::IMU> imu);
    dai::CameraControl setRuntimeParams(parametersConfig& config) override;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver