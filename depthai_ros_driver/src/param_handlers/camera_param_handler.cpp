#include "depthai_ros_driver/param_handlers/camera_param_handler.hpp"

#include "depthai/depthai.hpp"
#include "depthai/pipeline/nodes.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
CameraParamHandler::CameraParamHandler(const std::string& name) : BaseParamHandler(name) {
    usbSpeedMap = {
        {"LOW", dai::UsbSpeed::LOW},
        {"FULL", dai::UsbSpeed::FULL},
        {"HIGH", dai::UsbSpeed::HIGH},
        {"SUPER", dai::UsbSpeed::SUPER},
        {"SUPER_PLUS", dai::UsbSpeed::SUPER_PLUS},
    };
};
CameraParamHandler::~CameraParamHandler() = default;

dai::UsbSpeed CameraParamHandler::getUSBSpeed(rclcpp::Node* node) {
    return usbSpeedMap.at(getParam<std::string>(node, "i_usb_speed"));
}
void CameraParamHandler::declareParams(rclcpp::Node* node) {
    declareAndLogParam<std::string>(node, "i_pipeline_type", "RGBD");
    declareAndLogParam<std::string>(node, "i_nn_type", "spatial");
    declareAndLogParam<bool>(node, "i_enable_imu", true);
    declareAndLogParam<bool>(node, "i_enable_ir", true);
    declareAndLogParam<std::string>(node, "i_usb_speed", "SUPER_PLUS");
    declareAndLogParam<std::string>(node, "i_mx_id", "");
    declareAndLogParam<std::string>(node, "i_ip", "");
    declareAndLogParam<int>(node, "i_laser_dot_brightness", 800, getRangedIntDescriptor(0, 1200));
    declareAndLogParam<int>(node, "i_floodlight_brightness", 0, getRangedIntDescriptor(0, 1500));
}
dai::CameraControl CameraParamHandler::setRuntimeParams(rclcpp::Node* node, const std::vector<rclcpp::Parameter>& params) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver