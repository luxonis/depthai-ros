#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace dai {
namespace node {
class IMU;
}
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
namespace imu {
enum class ImuMsgType { IMU, IMU_WITH_MAG, IMU_WITH_MAG_SPLIT };
}
class ImuParamHandler : public BaseParamHandler {
   public:
    explicit ImuParamHandler(rclcpp::Node* node, const std::string& name);
    ~ImuParamHandler();
    void declareParams(std::shared_ptr<dai::node::IMU> imu, const std::string& imuType);
    dai::CameraControl setRuntimeParams(const std::vector<rclcpp::Parameter>& params) override;
    std::unordered_map<std::string, dai::ros::ImuSyncMethod> imuSyncMethodMap;
    std::unordered_map<std::string, imu::ImuMsgType> imuMessagetTypeMap;
    imu::ImuMsgType getMsgType();
    dai::ros::ImuSyncMethod getSyncMethod();
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver