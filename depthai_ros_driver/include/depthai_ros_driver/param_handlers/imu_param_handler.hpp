#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai/properties/IMUProperties.hpp"
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
    explicit ImuParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat);
    ~ImuParamHandler();
    void declareParams(std::shared_ptr<dai::node::IMU> imu, const std::string& imuType);
    std::unordered_map<std::string, depthai_bridge::ImuSyncMethod> syncMethodMap;
    std::unordered_map<std::string, imu::ImuMsgType> messagetTypeMap;
    std::unordered_map<std::string, dai::IMUSensor> rotationVectorTypeMap;
    std::unordered_map<std::string, dai::IMUSensor> accelerometerModeMap;
    std::unordered_map<std::string, dai::IMUSensor> gyroscopeModeMap;
    std::unordered_map<std::string, dai::IMUSensor> magnetometerModeMap;
    imu::ImuMsgType getMsgType();
    depthai_bridge::ImuSyncMethod getSyncMethod();
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
