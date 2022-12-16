#include "depthai_ros_driver/param_handlers/rgb_param_handler.hpp"

#include "depthai/depthai.hpp"
#include "depthai/pipeline/nodes.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
RGBParamHandler::RGBParamHandler(const std::string& name) : BaseParamHandler(name) {
    rgbResolutionMap = {
        {"1080", dai::ColorCameraProperties::SensorResolution::THE_1080_P},
        {"4k", dai::ColorCameraProperties::SensorResolution::THE_4_K},
        {"12MP", dai::ColorCameraProperties::SensorResolution::THE_12_MP},
    };
};
RGBParamHandler::~RGBParamHandler() = default;
void RGBParamHandler::declareParams(rclcpp::Node* node,
                                    std::shared_ptr<dai::node::ColorCamera> color_cam,
                                    dai::CameraBoardSocket socket,
                                    dai_nodes::sensor_helpers::ImageSensor sensor) {
    declareAndLogParam<int>(node, "i_max_q_size", 30);
    declareAndLogParam<bool>(node, "i_publish_topic", true);
    declareAndLogParam<bool>(node, "i_enable_preview", false);
    declareAndLogParam<int>(node, "i_board_socket_id", static_cast<int>(socket));
    color_cam->setBoardSocket(socket);
    color_cam->setFps(declareAndLogParam<double>(node, "i_fps", 30.0));
    size_t preview_size = declareAndLogParam<int>(node, "i_preview_size", 416);
    color_cam->setPreviewSize(preview_size, preview_size);
    auto resolution = rgbResolutionMap.at(declareAndLogParam<std::string>(node, "i_resolution", "1080"));
    int width, height;

    switch(resolution) {
        case dai::ColorCameraProperties::SensorResolution::THE_1080_P: {
            width = 1920;
            height = 1080;
            break;
        }
        case dai::ColorCameraProperties::SensorResolution::THE_4_K: {
            width = 3840;
            height = 2160;
            break;
        }
        case dai::ColorCameraProperties::SensorResolution::THE_12_MP: {
            height = 4056;
            width = 3040;
            break;
        }
    }
    color_cam->setResolution(resolution);

    color_cam->setInterleaved(declareAndLogParam<bool>(node, "i_interleaved", false));
    if(declareAndLogParam<bool>(node, "i_set_isp_scale", true)) {
        color_cam->setIspScale(2, 3);
        width = width * 2 / 3;
        height = height * 2 / 3;
    }
    color_cam->setVideoSize(declareAndLogParam<int>(node, "i_width", width), declareAndLogParam<int>(node, "i_height", height));
    color_cam->setPreviewKeepAspectRatio(declareAndLogParam(node, "i_keep_preview_aspect_ratio", true));
    size_t iso = declareAndLogParam(node, "r_iso", 800, getRangedIntDescriptor(100, 1600));
    size_t exposure = declareAndLogParam(node, "r_exposure", 20000, getRangedIntDescriptor(1, 33000));
    size_t whitebalance = declareAndLogParam(node, "r_whitebalance", 3300, getRangedIntDescriptor(1000, 12000));
    size_t focus = declareAndLogParam(node, "r_focus", 1, getRangedIntDescriptor(0, 255));
    if(declareAndLogParam(node, "r_set_man_focus", false)) {
        color_cam->initialControl.setManualFocus(focus);
    }
    if(declareAndLogParam(node, "r_set_man_exposure", false)) {
        color_cam->initialControl.setManualExposure(exposure, iso);
    }
    if(declareAndLogParam(node, "r_set_man_whitebalance", false)) {
        color_cam->initialControl.setManualWhiteBalance(whitebalance);
    }
}
dai::CameraControl RGBParamHandler::setRuntimeParams(rclcpp::Node* node, const std::vector<rclcpp::Parameter>& params) {
    dai::CameraControl ctrl;
    for(const auto& p : params) {
        if(p.get_name() == getFullParamName("r_set_man_exposure")) {
            if(p.get_value<bool>()) {
                ctrl.setManualExposure(getParam<int>(node, "r_exposure"), getParam<int>(node, "r_iso"));
            } else {
                ctrl.setAutoExposureEnable();
            }
        } else if(p.get_name() == getFullParamName("r_exposure")) {
            if(getParam<bool>(node, "r_set_man_exposure")) {
                ctrl.setManualExposure(p.get_value<int>(), getParam<int>(node, "r_iso"));
            }
        } else if(p.get_name() == getFullParamName("r_iso")) {
            if(getParam<bool>(node, "r_set_man_exposure")) {
                ctrl.setManualExposure(getParam<int>(node, "r_exposure"), p.get_value<int>());
            }
        } else if(p.get_name() == getFullParamName("r_set_man_focus")) {
            if(p.get_value<bool>()) {
                ctrl.setManualFocus(getParam<int>(node, "r_focus"));
            } else {
                ctrl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::CONTINUOUS_PICTURE);
            }
        } else if(p.get_name() == getFullParamName("r_focus")) {
            if(getParam<bool>(node, "r_set_man_focus")) {
                ctrl.setManualFocus(p.get_value<int>());
            }
        } else if(p.get_name() == getFullParamName("r_set_man_whitebalance")) {
            if(p.get_value<bool>()) {
                ctrl.setManualWhiteBalance(getParam<int>(node, "r_whitebalance"));
            } else {
                ctrl.setAutoWhiteBalanceMode(dai::CameraControl::AutoWhiteBalanceMode::AUTO);
            }
        } else if(p.get_name() == getFullParamName("r_whitebalance")) {
            if(getParam<bool>(node, "r_set_man_whitebalance")) {
                ctrl.setManualWhiteBalance(p.get_value<int>());
            }
        }

        return ctrl;
    }
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver