#include "depthai_ros_driver/param_handlers/camera_param_handler.hpp"

#include "depthai-shared/common/UsbSpeed.hpp"
#include "depthai_ros_driver/parametersConfig.h"
#include "depthai_ros_driver/utils.hpp"
#include "ros/node_handle.h"

namespace depthai_ros_driver {
namespace param_handlers {
CameraParamHandler::CameraParamHandler(ros::NodeHandle node, const std::string& name) : BaseParamHandler(node, name) {
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
    return utils::getValFromMap(getParam<std::string>("i_usb_speed"), usbSpeedMap);
}
void CameraParamHandler::declareParams() {
    getParam<std::string>("i_pipeline_type");
    getParam<std::string>("i_nn_type");
    getParam<bool>("i_enable_imu");
    getParam<bool>("i_enable_ir");
    getParam<std::string>("i_usb_speed");
    getParam<std::string>("i_mx_id");
    getParam<std::string>("i_ip");
    getParam<int>("i_laser_dot_brightness");
    getParam<int>("i_floodlight_brightness");
}
dai::CameraControl CameraParamHandler::setRuntimeParams(parametersConfig& /*config*/) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver