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
    nnTypeMap = {
        {"", camera::NNType::None},
        {"none", camera::NNType::None},
        {"rgb", camera::NNType::RGB},
        {"spatial", camera::NNType::Spatial},
    };
    pipelineTypeMap = {
        {"RGB", camera::PipelineType::RGB},
        {"RGBD", camera::PipelineType::RGBD}
    };
};
CameraParamHandler::~CameraParamHandler() = default;

camera::PipelineType CameraParamHandler::getPipelineType(rclcpp::Node* node) {
    auto type = getParam<std::string>(node, "i_pipeline_type");
    RCLCPP_INFO(node->get_logger(), "Pipeline type: %s", type.c_str());
    return pipelineTypeMap.at(type);
}

camera::NNType CameraParamHandler::getNNType(rclcpp::Node* node) {
    return nnTypeMap.at(getParam<std::string>(node, "i_nn_type"));
}
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
    declareAndLogParam<int>(node, "i_laser_dot_brightness", 0, getRangedIntDescriptor(0, 1200));
    declareAndLogParam<int>(node, "i_floodlight_brightness", 0, getRangedIntDescriptor(0, 1500));
}
dai::CameraControl CameraParamHandler::setRuntimeParams(rclcpp::Node* node, const std::vector<rclcpp::Parameter>& params) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver