#pragma once

#include <string>
#include <unordered_map>

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
class ImuParamHandler : public BaseParamHandler {
   public:
    explicit ImuParamHandler(const std::string& name);
    ~ImuParamHandler();
    void declareParams(ros::NodeHandle node, std::shared_ptr<dai::node::IMU> imu);
    dai::CameraControl setRuntimeParams(ros::NodeHandle node, parametersConfig& config) override;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver