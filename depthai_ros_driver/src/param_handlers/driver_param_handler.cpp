#include "depthai_ros_driver/param_handlers/driver_param_handler.hpp"

#include "depthai/common/UsbSpeed.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_value.hpp"

#include <stdexcept>

namespace depthai_ros_driver {
namespace param_handlers {
DriverParamHandler::DriverParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat)
    : BaseParamHandler(node, name, deviceName, rsCompat) {
    usbSpeedMap = {
        {"LOW", dai::UsbSpeed::LOW},
        {"FULL", dai::UsbSpeed::FULL},
        {"HIGH", dai::UsbSpeed::HIGH},
        {"SUPER", dai::UsbSpeed::SUPER},
        {"SUPER_PLUS", dai::UsbSpeed::SUPER_PLUS},
    };
}
DriverParamHandler::~DriverParamHandler() = default;

dai::UsbSpeed DriverParamHandler::getUSBSpeed() {
    return utils::getValFromMap(getParam<std::string>("i_usb_speed"), usbSpeedMap);
}
std::string DriverParamHandler::getPipelineAutoCalibrationMode() {
    rclcpp::Parameter param;
    getROSNode()->get_parameter(getFullParamName("i_pipeline_auto_calibration_mode"), param);
    if(param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        return param.as_string();
    }
    if(param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        return param.as_bool() ? "ON_START" : "OFF";
    }
    throw std::invalid_argument("Invalid driver.i_pipeline_auto_calibration_mode parameter type. Use OFF, ON_START, CONTINUOUS, empty, true, or false.");
}
void DriverParamHandler::declareParams() {
    declareAndLogParam<bool>("i_enable_ir", true);
    declareAndLogParam<std::string>("i_usb_speed", "SUPER");
    declareAndLogParam<std::string>("i_device_id", "");
    declareAndLogParam<std::string>("i_ip", "");
    declareAndLogParam<std::string>("i_usb_port_id", "");
    declareAndLogParam<bool>("i_pipeline_dump", false);
    declareAndLogParam<bool>("i_calibration_dump", false);
    declareAndLogParam<std::string>("i_external_calibration_path", "");
    auto autoCalibrationDescriptor = rcl_interfaces::msg::ParameterDescriptor();
    autoCalibrationDescriptor.dynamic_typing = true;
    if(!getROSNode()->has_parameter(getFullParamName("i_pipeline_auto_calibration_mode"))) {
        getROSNode()->declare_parameter(getFullParamName("i_pipeline_auto_calibration_mode"), rclcpp::ParameterValue(std::string()), autoCalibrationDescriptor);
    }
    declareAndLogParam<float>("r_laser_dot_intensity", 0.6, getRangedFloatDescriptor(0.0, 1.0));
    declareAndLogParam<float>("r_floodlight_intensity", 0.6, getRangedFloatDescriptor(0.0, 1.0));
    declareAndLogParam<bool>("i_restart_on_diagnostics_error", false);
    declareAndLogParam<bool>("i_rs_compat", false);

    declareAndLogParam<bool>("i_publish_tf_from_calibration", true);
    declareAndLogParam<std::string>("i_tf_device_name", getROSNode()->get_name());
    declareAndLogParam<std::string>("i_tf_device_model", "");
    declareAndLogParam<std::string>("i_tf_base_frame", "oak");
    declareAndLogParam<std::string>("i_tf_parent_frame", "oak_parent_frame");
    declareAndLogParam<std::string>("i_tf_cam_pos_x", "0.0");
    declareAndLogParam<std::string>("i_tf_cam_pos_y", "0.0");
    declareAndLogParam<std::string>("i_tf_cam_pos_z", "0.0");
    declareAndLogParam<std::string>("i_tf_cam_roll", "0.0");
    declareAndLogParam<std::string>("i_tf_cam_pitch", "0.0");
    declareAndLogParam<std::string>("i_tf_cam_yaw", "0.0");
    declareAndLogParam<std::string>("i_tf_imu_from_descr", "false");
    declareAndLogParam<std::string>("i_tf_custom_urdf_location", "");
    declareAndLogParam<std::string>("i_tf_custom_xacro_args", "");
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver
