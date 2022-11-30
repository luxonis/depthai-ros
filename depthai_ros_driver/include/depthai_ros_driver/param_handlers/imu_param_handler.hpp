#pragma once

#include <string>
#include <unordered_map>

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"


namespace depthai_ros_driver {
namespace paramHandlers {
class ImuParamHandler : public BaseParamHandler {
   public:
    explicit ImuParamHandler(const std::string& name);
    ~ImuParamHandler(){};
    void declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::IMU> imu);
    dai::CameraControl setRuntimeParams(rclcpp::Node* node,const std::vector<rclcpp::Parameter>& params) override;

};
}  // namespace paramHandlers
}  // namespace depthai_ros_driver