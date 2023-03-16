#include "depthai_ros_driver/param_handlers/rgb_param_handler.hpp"

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/properties/ColorCameraProperties.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
RGBParamHandler::RGBParamHandler(rclcpp::Node* node, const std::string& name) : BaseParamHandler(node, name) {
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
                        {"48_MP", dai::ColorCameraProperties::SensorResolution::THE_48_MP}};
}
RGBParamHandler::~RGBParamHandler() = default;
void RGBParamHandler::declareParams(std::shared_ptr<dai::node::ColorCamera> colorCam,
                                    dai::CameraBoardSocket socket,
                                    dai_nodes::sensor_helpers::ImageSensor sensor,
                                    bool publish) {
    declareAndLogParam<int>("i_max_q_size", 30);
    declareAndLogParam<bool>("i_publish_topic", publish);
    declareAndLogParam<bool>("i_enable_preview", false);
    declareAndLogParam<bool>("i_low_bandwidth", false);
    declareAndLogParam<int>("i_low_bandwidth_quality", 50);
    declareAndLogParam<int>("i_board_socket_id", static_cast<int>(socket));
    declareAndLogParam<std::string>("i_calibration_file", "");
    colorCam->setBoardSocket(socket);
    colorCam->setFps(declareAndLogParam<double>("i_fps", 30.0));
    size_t preview_size = declareAndLogParam<int>("i_preview_size", 300);
    colorCam->setPreviewSize(preview_size, preview_size);
    auto resolution = rgbResolutionMap.at(declareAndLogParam<std::string>("i_resolution", "1080"));
    int width, height;
    colorCam->setResolution(resolution);
    sensor.getSizeFromResolution(colorCam->getResolution(), width, height);

    colorCam->setInterleaved(declareAndLogParam<bool>("i_interleaved", false));
    if(declareAndLogParam<bool>("i_set_isp_scale", true)) {
        int new_width = width * 2 / 3;
        int new_height = height * 2 / 3;
        if(new_width % 16 == 0 && new_height % 16 == 0) {
            width = new_width;
            height = new_height;
            colorCam->setIspScale(2, 3);
        } else {
            RCLCPP_ERROR(getROSNode()->get_logger(), "ISP scaling not supported for given width & height");
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
}
dai::CameraControl RGBParamHandler::setRuntimeParams(const std::vector<rclcpp::Parameter>& params) {
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