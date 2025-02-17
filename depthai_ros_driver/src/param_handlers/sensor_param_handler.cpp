#include "depthai_ros_driver/param_handlers/sensor_param_handler.hpp"

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/CameraFeatures.hpp"
#include "depthai-shared/properties/ColorCameraProperties.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "ros/node_handle.h"

namespace depthai_ros_driver {
namespace param_handlers {
SensorParamHandler::SensorParamHandler(ros::NodeHandle node, const std::string& name, dai::CameraBoardSocket socket) : BaseParamHandler(node, name) {
    declareCommonParams(socket);
}
SensorParamHandler::~SensorParamHandler() = default;

void SensorParamHandler::declareCommonParams(dai::CameraBoardSocket socket) {
    declareAndLogParam<int>("i_max_q_size", 30);
    declareAndLogParam<bool>("i_low_bandwidth", false);
    declareAndLogParam<int>("i_low_bandwidth_profile", 4);
    declareAndLogParam<int>("i_low_bandwidth_frame_freq", 30);
    declareAndLogParam<int>("i_low_bandwidth_bitrate", 0);
    declareAndLogParam<int>("i_low_bandwidth_quality", 50);
    declareAndLogParam<std::string>("i_low_bandwidth_ffmpeg_encoder", "libx264");
    declareAndLogParam<std::string>("i_calibration_file", "");
    declareAndLogParam<bool>("i_simulate_from_topic", false);
    declareAndLogParam<std::string>("i_simulated_topic_name", "");
    declareAndLogParam<bool>("i_disable_node", false);
    declareAndLogParam<bool>("i_get_base_device_timestamp", false);
    socketID = static_cast<dai::CameraBoardSocket>(declareAndLogParam<int>("i_board_socket_id", static_cast<int>(socket), false));
    declareAndLogParam<bool>("i_update_ros_base_time_on_ros_msg", false);
    declareAndLogParam<bool>("i_enable_feature_tracker", false);
    declareAndLogParam<bool>("i_enable_nn", false);
    declareAndLogParam<bool>("i_enable_lazy_publisher", true);
    declareAndLogParam<bool>("i_add_exposure_offset", false);
    declareAndLogParam<int>("i_exposure_offset", 0);
    declareAndLogParam<bool>("i_reverse_stereo_socket_order", false);
    declareAndLogParam<bool>("i_synced", false);
    declareAndLogParam<bool>("i_publish_compressed", false);
}

void SensorParamHandler::declareParams(std::shared_ptr<dai::node::Camera> cam, dai::CameraFeatures features, bool publish) {
    cam->setBoardSocket(socketID);
    cam->setFps(declareAndLogParam<double>("i_fps", 30.0));
    declareAndLogParam<bool>("i_publish_topic", publish);
    int width = declareAndLogParam<int>("i_width", features.width);
    int height = declareAndLogParam<int>("i_height", features.height);
    declareAndLogParam<bool>("i_publish_raw", true);
    cam->setPreviewSize(width, height);
}

void SensorParamHandler::declareParams(std::shared_ptr<dai::node::MonoCamera> monoCam, dai_nodes::sensor_helpers::ImageSensor sensor, bool publish) {
    monoCam->setBoardSocket(socketID);
    monoCam->setFps(declareAndLogParam<double>("i_fps", 30.0));
    declareAndLogParam<bool>("i_publish_topic", publish);

    auto resString = declareAndLogParam<std::string>("i_resolution", sensor.defaultResolution);

    // if resolution not in allowed resolutions, use default
    if(std::find(sensor.allowedResolutions.begin(), sensor.allowedResolutions.end(), resString) == sensor.allowedResolutions.end()) {
        ROS_WARN(
            "Resolution %s not supported by sensor %s. Using default resolution %s", resString.c_str(), sensor.name.c_str(), sensor.defaultResolution.c_str());
        resString = sensor.defaultResolution;
    }

    monoCam->setResolution(utils::getValFromMap(resString, dai_nodes::sensor_helpers::monoResolutionMap));
    declareAndLogParam<int>("i_width", monoCam->getResolutionWidth());
    declareAndLogParam<int>("i_height", monoCam->getResolutionHeight());
    size_t iso = declareAndLogParam("r_iso", 800, getRangedIntDescriptor(100, 1600));
    size_t exposure = declareAndLogParam("r_exposure", 1000, getRangedIntDescriptor(1, 33000));

    if(declareAndLogParam<bool>("r_set_man_exposure", false)) {
        monoCam->initialControl.setManualExposure(exposure, iso);
    }
    if(declareAndLogParam<bool>("i_fsync_continuous", false)) {
        monoCam->initialControl.setFrameSyncMode(
            utils::getValFromMap(declareAndLogParam<std::string>("i_fsync_mode", "INPUT"), dai_nodes::sensor_helpers::fSyncModeMap));
    }
    if(declareAndLogParam<bool>("i_fsync_trigger", false)) {
        monoCam->initialControl.setExternalTrigger(declareAndLogParam<int>("i_num_frames_burst", 1), declareAndLogParam<int>("i_num_frames_discard", 0));
    }
    if(declareAndLogParam<bool>("i_set_isp3a_fps", false)) {
        monoCam->setIsp3aFps(declareAndLogParam<int>("i_isp3a_fps", 10));
    }
    monoCam->setImageOrientation(
        utils::getValFromMap(declareAndLogParam<std::string>("i_sensor_img_orientation", "AUTO"), dai_nodes::sensor_helpers::cameraImageOrientationMap));
    int expLimit = declareAndLogParam<int>("r_auto_exposure_limit", 1000);
    if(declareAndLogParam<bool>("r_set_auto_exposure_limit", false)) {
        monoCam->initialControl.setAutoExposureLimit(expLimit);
    }
    int sharpness = declareAndLogParam<int>("r_sharpness", 1);
    if(declareAndLogParam("r_set_sharpness", false)) {
        monoCam->initialControl.setSharpness(sharpness);
    }
    int chromaDenoise = declareAndLogParam<int>("r_chroma_denoise", 1);
    if(declareAndLogParam("r_set_chroma_denoise", false)) {
        monoCam->initialControl.setChromaDenoise(chromaDenoise);
    }
    int lumaDenoise = declareAndLogParam<int>("r_luma_denoise", 1);
    if(declareAndLogParam("r_set_luma_denoise", false)) {
        monoCam->initialControl.setLumaDenoise(lumaDenoise);
    }
}

void SensorParamHandler::declareParams(std::shared_ptr<dai::node::ColorCamera> colorCam, dai_nodes::sensor_helpers::ImageSensor sensor, bool publish) {
    declareAndLogParam<bool>("i_publish_topic", publish);
    declareAndLogParam<bool>("i_output_isp", true);
    declareAndLogParam<bool>("i_enable_preview", false);
    colorCam->setBoardSocket(socketID);
    colorCam->setFps(declareAndLogParam<double>("i_fps", 30.0));
    int preview_size = declareAndLogParam<int>("i_preview_size", 300);
    int preview_width = declareAndLogParam<int>("i_preview_width", preview_size);
    int preview_height = declareAndLogParam<int>("i_preview_height", preview_size);
    colorCam->setPreviewSize(preview_width, preview_height);
    auto resString = declareAndLogParam<std::string>("i_resolution", sensor.defaultResolution);

    // if resolution not in allowed resolutions, use default
    if(std::find(sensor.allowedResolutions.begin(), sensor.allowedResolutions.end(), resString) == sensor.allowedResolutions.end()) {
        ROS_WARN(
            "Resolution %s not supported by sensor %s. Using default resolution %s", resString.c_str(), sensor.name.c_str(), sensor.defaultResolution.c_str());
        resString = sensor.defaultResolution;
    }

    colorCam->setResolution(utils::getValFromMap(resString, dai_nodes::sensor_helpers::rgbResolutionMap));
    int width = colorCam->getResolutionWidth();
    int height = colorCam->getResolutionHeight();

    colorCam->setInterleaved(declareAndLogParam<bool>("i_interleaved", false));

    bool setIspScale = true;
    if(sensor.defaultResolution != "1080P"
       && sensor.defaultResolution != "1200P") {  // default disable ISP scaling since default resolution is not 1080P or 1200P
        setIspScale = false;
    }
    if(declareAndLogParam<bool>("i_set_isp_scale", setIspScale)) {
        int num = 2;
        int den = 3;
        if(sensor.defaultResolution == "1200P") {
            den = 5;  // for improved performance
        }
        num = declareAndLogParam<int>("i_isp_num", num);
        den = declareAndLogParam<int>("i_isp_den", den);
        width = (width * num + den - 1) / den;
        height = (height * num + den - 1) / den;
        colorCam->setIspScale(num, den);
        if(width % 16 != 0 && height % 16 != 0) {
            std::stringstream err_stream;
            err_stream << "ISP scaling with num: " << num << " and den: " << den << " results in width: " << width << " and height: " << height;
            err_stream << " which are not divisible by 16.\n";
            err_stream << "This will result in errors when aligning stereo to RGB. To fix that, either adjust i_num and i_den values";
            err_stream << " or set i_output_isp parameter to false and set i_width and i_height parameters accordingly.";
            ROS_ERROR(err_stream.str().c_str());
        }
    }
    int maxVideoWidth = 3840;
    int maxVideoHeight = 2160;
    int videoWidth = declareAndLogParam<int>("i_width", width);
    int videoHeight = declareAndLogParam<int>("i_height", height);
    if(videoWidth > maxVideoWidth) {
        ROS_WARN("Video width %d is greater than max video width %d. Setting video width to max video width.", videoWidth, maxVideoWidth);
        videoWidth = maxVideoWidth;
    }
    if(videoHeight > maxVideoHeight) {
        ROS_WARN("Video height %d is greater than max video height %d. Setting video height to max video height.", videoHeight, maxVideoHeight);
        videoHeight = maxVideoHeight;
    }
    colorCam->setVideoSize(videoWidth, videoHeight);
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
        colorCam->initialControl.setFrameSyncMode(
            utils::getValFromMap(declareAndLogParam<std::string>("i_fsync_mode", "INPUT"), dai_nodes::sensor_helpers::fSyncModeMap));
    }
    if(declareAndLogParam<bool>("i_fsync_trigger", false)) {
        colorCam->initialControl.setExternalTrigger(declareAndLogParam<int>("i_num_frames_burst", 1), declareAndLogParam<int>("i_num_frames_discard", 0));
    }
    if(declareAndLogParam<bool>("i_set_isp3a_fps", false)) {
        colorCam->setIsp3aFps(declareAndLogParam<int>("i_isp3a_fps", 10));
    }
    colorCam->setImageOrientation(
        utils::getValFromMap(declareAndLogParam<std::string>("i_sensor_img_orientation", "AUTO"), dai_nodes::sensor_helpers::cameraImageOrientationMap));
    int expLimit = declareAndLogParam<int>("r_auto_exposure_limit", 1000);
    if(declareAndLogParam<bool>("r_set_auto_exposure_limit", false)) {
        colorCam->initialControl.setAutoExposureLimit(expLimit);
    }
    int sharpness = declareAndLogParam<int>("r_sharpness", 1);
    if(declareAndLogParam("r_set_sharpness", false)) {
        colorCam->initialControl.setSharpness(sharpness);
    }
    int chromaDenoise = declareAndLogParam<int>("r_chroma_denoise", 1);
    if(declareAndLogParam("r_set_chroma_denoise", false)) {
        colorCam->initialControl.setChromaDenoise(chromaDenoise);
    }
    int lumaDenoise = declareAndLogParam<int>("r_luma_denoise", 1);
    if(declareAndLogParam("r_set_luma_denoise", false)) {
        colorCam->initialControl.setLumaDenoise(lumaDenoise);
    }
}
dai::CameraControl SensorParamHandler::setRuntimeParams(parametersConfig& config) {
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
        if(config.rgb_r_set_auto_exposure_limit) {
            ctrl.setAutoExposureLimit(config.rgb_r_auto_exposure_limit);
        }
        if(config.rgb_r_set_sharpness) {
            ctrl.setSharpness(config.rgb_r_sharpness);
        }
        if(config.rgb_r_set_chroma_denoise) {
            ctrl.setChromaDenoise(config.rgb_r_chroma_denoise);
        }
        if(config.rgb_r_set_luma_denoise) {
            ctrl.setLumaDenoise(config.rgb_r_luma_denoise);
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
        if(config.left_r_set_auto_exposure_limit) {
            ctrl.setAutoExposureLimit(config.left_r_auto_exposure_limit);
        }
        if(config.left_r_set_sharpness) {
            ctrl.setSharpness(config.left_r_sharpness);
        }
        if(config.left_r_set_chroma_denoise) {
            ctrl.setChromaDenoise(config.left_r_chroma_denoise);
        }
        if(config.left_r_set_luma_denoise) {
            ctrl.setLumaDenoise(config.left_r_luma_denoise);
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
        if(config.right_r_set_auto_exposure_limit) {
            ctrl.setAutoExposureLimit(config.right_r_auto_exposure_limit);
        }
        if(config.right_r_set_sharpness) {
            ctrl.setSharpness(config.right_r_sharpness);
        }
        if(config.right_r_set_chroma_denoise) {
            ctrl.setChromaDenoise(config.right_r_chroma_denoise);
        }
        if(config.right_r_set_luma_denoise) {
            ctrl.setLumaDenoise(config.right_r_luma_denoise);
        }
    }

    return ctrl;
}

}  // namespace param_handlers
}  // namespace depthai_ros_driver
