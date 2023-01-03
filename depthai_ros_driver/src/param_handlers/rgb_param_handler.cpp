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
void RGBParamHandler::declareParams(ros::NodeHandle node,
                                    std::shared_ptr<dai::node::ColorCamera> color_cam,
                                    dai::CameraBoardSocket socket,
                                    dai_nodes::sensor_helpers::ImageSensor sensor) {
    getParam<int>(node, "i_max_q_size");
    getParam<bool>(node, "i_publish_topic");
    getParam<bool>(node, "i_enable_preview");
    getParam<int>(node, "i_board_socket_id");

    color_cam->setBoardSocket(socket);
    color_cam->setFps(getParam<int>(node, "i_fps"));
    color_cam->setPreviewSize(getParam<int>(node, "i_preview_size"), getParam<int>(node, "i_preview_size"));
    auto resolution = rgbResolutionMap.at(getParam<std::string>(node, "i_resolution"));
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

    color_cam->setInterleaved(getParam<bool>(node, "i_interleaved"));
    if(getParam<bool>(node, "i_set_isp_scale")) {
        color_cam->setIspScale(2, 3);
        width = width * 2 / 3;
        height = height * 2 / 3;
    }
    color_cam->setVideoSize(width, height);

    color_cam->setPreviewKeepAspectRatio(getParam<bool>(node, "i_keep_preview_aspect_ratio"));
    size_t iso = getParam<int>(node, "r_iso");
    size_t exposure = getParam<int>(node, "r_exposure");
    size_t whitebalance = getParam<int>(node, "r_whitebalance");
    size_t focus = getParam<int>(node, "r_focus");
    if(getParam<bool>(node, "r_set_man_focus")) {
        color_cam->initialControl.setManualFocus(focus);
    }
    if(getParam<bool>(node, "r_set_man_exposure")) {
        color_cam->initialControl.setManualExposure(exposure, iso);
    }
    if(getParam<bool>(node, "r_set_man_whitebalance")) {
        color_cam->initialControl.setManualWhiteBalance(whitebalance);
    }
}
dai::CameraControl RGBParamHandler::setRuntimeParams(ros::NodeHandle node, parametersConfig& config) {
    dai::CameraControl ctrl;

    if(getName() == "rgb") {
        if(config.rgb_r_set_man_exposure) {
            ctrl.setManualExposure(config.rgb_r_exposure, config.rgb_r_iso);
        } else {
            ctrl.setAutoExposureEnable();
        }

        if(config.rgb_r_set_man_focus) {
            ctrl.setManualFocus(config.rgb_r_focus);
        } else {
            ctrl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::CONTINUOUS_PICTURE);
        }
        if(config.rgb_r_set_man_whitebalance) {
            ctrl.setManualWhiteBalance(config.rgb_r_whitebalance);
        } else {
            ctrl.setAutoWhiteBalanceMode(dai::CameraControl::AutoWhiteBalanceMode::AUTO);
        }
    } else if(getName() == "left") {
        if(config.left_r_set_man_exposure) {
            ctrl.setManualExposure(config.left_r_exposure, config.left_r_iso);
        } else {
            ctrl.setAutoExposureEnable();
        }

        if(config.left_r_set_man_focus) {
            ctrl.setManualFocus(config.left_r_focus);
        } else {
            ctrl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::CONTINUOUS_PICTURE);
        }
        if(config.left_r_set_man_whitebalance) {
            ctrl.setManualWhiteBalance(config.left_r_whitebalance);
        } else {
            ctrl.setAutoWhiteBalanceMode(dai::CameraControl::AutoWhiteBalanceMode::AUTO);
        }
    } else if(getName() == "right") {
        if(config.right_r_set_man_exposure) {
            ctrl.setManualExposure(config.right_r_exposure, config.right_r_iso);
        } else {
            ctrl.setAutoExposureEnable();
        }

        if(config.right_r_set_man_focus) {
            ctrl.setManualFocus(config.right_r_focus);
        } else {
            ctrl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::CONTINUOUS_PICTURE);
        }
        if(config.right_r_set_man_whitebalance) {
            ctrl.setManualWhiteBalance(config.right_r_whitebalance);
        } else {
            ctrl.setAutoWhiteBalanceMode(dai::CameraControl::AutoWhiteBalanceMode::AUTO);
        }
    }

    return ctrl;
}

}  // namespace param_handlers
}  // namespace depthai_ros_driver