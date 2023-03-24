#include "depthai_ros_driver/param_handlers/camera_param_handler.hpp"

#include "depthai-shared/common/UsbSpeed.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
CameraParamHandler::CameraParamHandler(rclcpp::Node* node, const std::string& name) : BaseParamHandler(node, name) {
    usbSpeedMap = {
        {"LOW", dai::UsbSpeed::LOW},
        {"FULL", dai::UsbSpeed::FULL},
        {"HIGH", dai::UsbSpeed::HIGH},
        {"SUPER", dai::UsbSpeed::SUPER},
        {"SUPER_PLUS", dai::UsbSpeed::SUPER_PLUS},
    };
}
CameraParamHandler::~CameraParamHandler() = default;

dai::UsbSpeed CameraParamHandler::getUSBSpeed() {
    return usbSpeedMap.at(getParam<std::string>("i_usb_speed"));
}
void CameraParamHandler::declareParams() {
    declareAndLogParam<std::string>("i_pipeline_type", "RGBD");
    declareAndLogParam<std::string>("i_nn_type", "spatial");
    declareAndLogParam<bool>("i_enable_imu", true);
    declareAndLogParam<bool>("i_enable_ir", true);
    declareAndLogParam<std::string>("i_usb_speed", "SUPER_PLUS");
    declareAndLogParam<std::string>("i_mx_id", "");
    declareAndLogParam<std::string>("i_ip", "");
    declareAndLogParam<std::string>("i_usb_port_id", "");
    declareAndLogParam<int>("i_laser_dot_brightness", 800, getRangedIntDescriptor(0, 1200));
    declareAndLogParam<int>("i_floodlight_brightness", 0, getRangedIntDescriptor(0, 1500));
}
dai::CameraControl CameraParamHandler::setRuntimeParams(const std::vector<rclcpp::Parameter>& /*params*/) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver