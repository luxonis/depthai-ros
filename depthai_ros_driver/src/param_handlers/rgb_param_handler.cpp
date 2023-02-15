#include "depthai_ros_driver/param_handlers/rgb_param_handler.hpp"

#include "depthai/depthai.hpp"
#include "depthai/pipeline/nodes.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
RGBParamHandler::RGBParamHandler(const std::string& name) : BaseParamHandler(name) {
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
void RGBParamHandler::declareParams(ros::NodeHandle node,
                                    std::shared_ptr<dai::node::ColorCamera> colorCam,
                                    dai::CameraBoardSocket socket,
                                    dai_nodes::sensor_helpers::ImageSensor sensor,
                                    bool publish) {
    getParam<int>(node, "i_max_q_size", 30);
    getParam<bool>(node, "i_publish_topic", publish);
    getParam<bool>(node, "i_enable_preview", false);
    getParam<int>(node, "i_board_socket_id", static_cast<int>(socket));

    colorCam->setBoardSocket(socket);
    colorCam->setFps(getParam<int>(node, "i_fps", 30.0));
    colorCam->setPreviewSize(getParam<int>(node, "i_preview_size", 416), getParam<int>(node, "i_preview_size", 416));
    auto resolution = rgbResolutionMap.at(getParam<std::string>(node, "i_resolution", "1080"));
    int width, height;
    colorCam->setResolution(resolution);
    sensor.getSizeFromResolution(colorCam->getResolution(), width, height);

    colorCam->setInterleaved(getParam<bool>(node, "i_interleaved", false));
    if(getParam<bool>(node, "i_set_isp_scale", true)) {
        int new_width = width * 2 / 3;
        int new_height = height * 2 / 3;
        if(new_width % 16 == 0 && new_height % 16 == 0) {
            width = new_width;
            height = new_height;
            colorCam->setIspScale(2, 3);
        } else {
            ROS_ERROR("ISP scaling not supported for given width & height");
        }
    }
    colorCam->setVideoSize(width, height);
    setParam<int>(node, "i_height", height);
    setParam<int>(node, "i_width", width);

    colorCam->setPreviewKeepAspectRatio(getParam<bool>(node, "i_keep_preview_aspect_ratio", true));
    size_t iso = getParam<int>(node, "r_iso", 800);
    size_t exposure = getParam<int>(node, "r_exposure", 20000);
    size_t whitebalance = getParam<int>(node, "r_whitebalance", 3300);
    size_t focus = getParam<int>(node, "r_focus", 1);
    if(getParam<bool>(node, "r_set_man_focus", false)) {
        colorCam->initialControl.setManualFocus(focus);
    }
    if(getParam<bool>(node, "r_set_man_exposure", false)) {
        colorCam->initialControl.setManualExposure(exposure, iso);
    }
    if(getParam<bool>(node, "r_set_man_whitebalance", false)) {
        colorCam->initialControl.setManualWhiteBalance(whitebalance);
    }
}
dai::CameraControl RGBParamHandler::setRuntimeParams(ros::NodeHandle /*node*/, parametersConfig& config) {
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