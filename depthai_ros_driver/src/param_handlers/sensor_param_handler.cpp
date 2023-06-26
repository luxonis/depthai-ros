#include "depthai_ros_driver/param_handlers/sensor_param_handler.hpp"

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/properties/ColorCameraProperties.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
SensorParamHandler::SensorParamHandler(rclcpp::Node* node, const std::string& name) : BaseParamHandler(node, name) {
    declareCommonParams();
};
SensorParamHandler::~SensorParamHandler() = default;

void SensorParamHandler::declareCommonParams() {
    declareAndLogParam<int>("i_max_q_size", 30);
    declareAndLogParam<bool>("i_low_bandwidth", false);
    declareAndLogParam<int>("i_low_bandwidth_quality", 50);
    declareAndLogParam<std::string>("i_calibration_file", "");
    declareAndLogParam<bool>("i_simulate_from_topic", false);
    declareAndLogParam<std::string>("i_simulated_topic_name", "");
    declareAndLogParam<bool>("i_disable_node", false);
    declareAndLogParam<bool>("i_get_base_device_timestamp", false);
    declareAndLogParam<int>("i_board_socket_id", 0);
    declareAndLogParam<bool>("i_update_ros_base_time_on_ros_msg", false);
    fSyncModeMap = {
        {"OFF", dai::CameraControl::FrameSyncMode::OFF},
        {"OUTPUT", dai::CameraControl::FrameSyncMode::OUTPUT},
        {"INPUT", dai::CameraControl::FrameSyncMode::INPUT},
    };
}

void SensorParamHandler::declareParams(std::shared_ptr<dai::node::MonoCamera> monoCam,
                                       dai::CameraBoardSocket socket,
                                       dai_nodes::sensor_helpers::ImageSensor,
                                       bool publish) {
    monoResolutionMap = {
        {"400", dai::MonoCameraProperties::SensorResolution::THE_400_P},
        {"480", dai::MonoCameraProperties::SensorResolution::THE_480_P},
        {"720", dai::MonoCameraProperties::SensorResolution::THE_720_P},
        {"800", dai::MonoCameraProperties::SensorResolution::THE_800_P},
        {"1200", dai::MonoCameraProperties::SensorResolution::THE_1200_P},
    };
    declareAndLogParam<int>("i_board_socket_id", static_cast<int>(socket), true);
    monoCam->setBoardSocket(socket);
    monoCam->setFps(declareAndLogParam<double>("i_fps", 30.0));
    declareAndLogParam<bool>("i_publish_topic", publish);
    monoCam->setResolution(utils::getValFromMap(declareAndLogParam<std::string>("i_resolution", "720"), monoResolutionMap));
    declareAndLogParam<int>("i_width", monoCam->getResolutionWidth());
    declareAndLogParam<int>("i_height", monoCam->getResolutionHeight());
    size_t iso = declareAndLogParam("r_iso", 800, getRangedIntDescriptor(100, 1600));
    size_t exposure = declareAndLogParam("r_exposure", 1000, getRangedIntDescriptor(1, 33000));

    if(declareAndLogParam<bool>("r_set_man_exposure", false)) {
        monoCam->initialControl.setManualExposure(exposure, iso);
    }
    if(declareAndLogParam<bool>("i_fsync_continuous", false)) {
        monoCam->initialControl.setFrameSyncMode(utils::getValFromMap(declareAndLogParam<std::string>("i_fsync_mode", "INPUT"), fSyncModeMap));
    }
    if(declareAndLogParam<bool>("i_fsync_trigger", false)) {
        monoCam->initialControl.setExternalTrigger(declareAndLogParam<int>("i_num_frames_burst", 1), declareAndLogParam<int>("i_num_frames_discard", 0));
    }
}
void SensorParamHandler::declareParams(std::shared_ptr<dai::node::ColorCamera> colorCam,
                                       dai::CameraBoardSocket socket,
                                       dai_nodes::sensor_helpers::ImageSensor sensor,
                                       bool publish) {
    rgbResolutionMap = {{"720", dai::ColorCameraProperties::SensorResolution::THE_720_P},
                        {"1080", dai::ColorCameraProperties::SensorResolution::THE_1080_P},
                        {"4K", dai::ColorCameraProperties::SensorResolution::THE_4_K},
                        {"12MP", dai::ColorCameraProperties::SensorResolution::THE_12_MP},
                        {"13MP", dai::ColorCameraProperties::SensorResolution::THE_13_MP},
                        {"800", dai::ColorCameraProperties::SensorResolution::THE_800_P},
                        {"1200", dai::ColorCameraProperties::SensorResolution::THE_1200_P},
                        {"5MP", dai::ColorCameraProperties::SensorResolution::THE_5_MP},
                        {"4000x3000", dai::ColorCameraProperties::SensorResolution::THE_4000X3000},
                        {"5312X6000", dai::ColorCameraProperties::SensorResolution::THE_5312X6000},
                        {"48_MP", dai::ColorCameraProperties::SensorResolution::THE_48_MP},
                        {"1440X1080", dai::ColorCameraProperties::SensorResolution::THE_1440X1080}};
    declareAndLogParam<bool>("i_publish_topic", publish);
    declareAndLogParam<int>("i_board_socket_id", static_cast<int>(socket));
    declareAndLogParam<bool>("i_output_isp", true);
    declareAndLogParam<bool>("i_enable_preview", false);
    colorCam->setBoardSocket(socket);
    colorCam->setFps(declareAndLogParam<double>("i_fps", 30.0));
    size_t preview_size = declareAndLogParam<int>("i_preview_size", 300);
    colorCam->setPreviewSize(preview_size, preview_size);
    auto resolution = utils::getValFromMap(declareAndLogParam<std::string>("i_resolution", "1080"), rgbResolutionMap);
    int width, height;
    colorCam->setResolution(resolution);
    sensor.getSizeFromResolution(colorCam->getResolution(), width, height);

    colorCam->setInterleaved(declareAndLogParam<bool>("i_interleaved", false));
    if(declareAndLogParam<bool>("i_set_isp_scale", true)) {
        int num = declareAndLogParam<int>("i_isp_num", 2);
        int den = declareAndLogParam<int>("i_isp_den", 3);
        width = (width * num + den - 1) / den;
        height = (height * num + den - 1) / den;
        colorCam->setIspScale(num, den);
        if(width % 16 != 0 && height % 16 != 0) {
            std::stringstream err_stream;
            err_stream << "ISP scaling with num: " << num << " and den: " << den << " results in width: " << width << " and height: " << height;
            err_stream << " which are not divisible by 16.\n";
            err_stream << "This will result in errors when aligning stereo to RGB. To fix that, either adjust i_num and i_den values";
            err_stream << " or set i_output_isp parameter to false and set i_width and i_height parameters accordingly.";
            RCLCPP_ERROR(getROSNode()->get_logger(), err_stream.str().c_str());
        }
    }
    colorCam->setVideoSize(declareAndLogParam<int>("i_width", width), declareAndLogParam<int>("i_height", height));
    colorCam->setPreviewKeepAspectRatio(declareAndLogParam("i_keep_preview_aspect_ratio", true));
    size_t iso = declareAndLogParam("r_iso", 800, getRangedIntDescriptor(100, 1600));
    size_t exposure = declareAndLogParam("r_exposure", 20000, getRangedIntDescriptor(1, 33000));
    size_t whitebalance = declareAndLogParam("r_whitebalance", 3300, getRangedIntDescriptor(1000, 12000));
    size_t focus = declareAndLogParam("r_focus", 1, getRangedIntDescriptor(0, 255));
    if(declareAndLogParam("r_set_man_focus", false)) {
        colorCam->initialControl.setManualFocus(focus);
    }
    if(declareAndLogParam("r_set_man_exposure", false)) {
        colorCam->initialControl.setManualExposure(exposure, iso);
    }
    if(declareAndLogParam("r_set_man_whitebalance", false)) {
        colorCam->initialControl.setManualWhiteBalance(whitebalance);
    }
    if(declareAndLogParam<bool>("i_fsync_continuous", false)) {
        colorCam->initialControl.setFrameSyncMode(utils::getValFromMap(declareAndLogParam<std::string>("i_fsync_mode", "INPUT"), fSyncModeMap));
    }
    if(declareAndLogParam<bool>("i_fsync_trigger", false)) {
        colorCam->initialControl.setExternalTrigger(declareAndLogParam<int>("i_num_frames_burst", 1), declareAndLogParam<int>("i_num_frames_discard", 0));
    }
}
dai::CameraControl SensorParamHandler::setRuntimeParams(const std::vector<rclcpp::Parameter>& params) {
    dai::CameraControl ctrl;
    for(const auto& p : params) {
        if(p.get_name() == getFullParamName("r_set_man_exposure")) {
            if(p.get_value<bool>()) {
                ctrl.setManualExposure(getParam<int>("r_exposure"), getParam<int>("r_iso"));
            } else {
                ctrl.setAutoExposureEnable();
            }
        } else if(p.get_name() == getFullParamName("r_exposure")) {
            if(getParam<bool>("r_set_man_exposure")) {
                ctrl.setManualExposure(p.get_value<int>(), getParam<int>("r_iso"));
            }
        } else if(p.get_name() == getFullParamName("r_iso")) {
            if(getParam<bool>("r_set_man_exposure")) {
                ctrl.setManualExposure(getParam<int>("r_exposure"), p.get_value<int>());
            }
        } else if(p.get_name() == getFullParamName("r_set_man_focus")) {
            if(p.get_value<bool>()) {
                ctrl.setManualFocus(getParam<int>("r_focus"));
            } else {
                ctrl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::CONTINUOUS_PICTURE);
            }
        } else if(p.get_name() == getFullParamName("r_focus")) {
            if(getParam<bool>("r_set_man_focus")) {
                ctrl.setManualFocus(p.get_value<int>());
            }
        } else if(p.get_name() == getFullParamName("r_set_man_whitebalance")) {
            if(p.get_value<bool>()) {
                ctrl.setManualWhiteBalance(getParam<int>("r_whitebalance"));
            } else {
                ctrl.setAutoWhiteBalanceMode(dai::CameraControl::AutoWhiteBalanceMode::AUTO);
            }
        } else if(p.get_name() == getFullParamName("r_whitebalance")) {
            if(getParam<bool>("r_set_man_whitebalance")) {
                ctrl.setManualWhiteBalance(p.get_value<int>());
            }
        }
    }
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver