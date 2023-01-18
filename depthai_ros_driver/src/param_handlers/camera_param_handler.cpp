#include "depthai_ros_driver/param_handlers/camera_param_handler.hpp"

#include "depthai/depthai.hpp"
#include "depthai/pipeline/nodes.hpp"
#include "depthai_ros_driver/parametersConfig.h"

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
}
CameraParamHandler::~CameraParamHandler() = default;

dai::UsbSpeed CameraParamHandler::getUSBSpeed(ros::NodeHandle node) {
    return usbSpeedMap.at(getParam<std::string>(node, "i_usb_speed"));
}
void CameraParamHandler::declareParams(ros::NodeHandle node) {
    getParam<std::string>(node, "i_pipeline_type");
    getParam<std::string>(node, "i_nn_type");
    getParam<bool>(node, "i_enable_imu");
    getParam<bool>(node, "i_enable_ir");
    getParam<std::string>(node, "i_usb_speed");
    getParam<std::string>(node, "i_mx_id");
    getParam<std::string>(node, "i_ip");
    getParam<int>(node, "i_laser_dot_brightness");
    getParam<int>(node, "i_floodlight_brightness");
}
dai::CameraControl CameraParamHandler::setRuntimeParams(ros::NodeHandle /*node*/, parametersConfig& /*config*/) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver