#include "depthai_ros_driver/param_handlers/sensor_param_handler.hpp"

#include <memory>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
SensorParamHandler::SensorParamHandler(
    std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat, dai::CameraBoardSocket socket)
    : BaseParamHandler(node, name, deviceName, rsCompat) {
    declareCommonParams(socket);
};
SensorParamHandler::~SensorParamHandler() = default;

void SensorParamHandler::declareCommonParams(dai::CameraBoardSocket socket) {
    declareAndLogParam<bool>(ParamNames::SIMULATE_FROM_TOPIC, false);
    declareAndLogParam<std::string>(ParamNames::SIMULATED_TOPIC_NAME, "");
    declareAndLogParam<bool>(ParamNames::DISABLE_NODE, false);
    socketID = static_cast<dai::CameraBoardSocket>(declareAndLogParam<int>(ParamNames::BOARD_SOCKET_ID, static_cast<int>(socket)));
    declareAndLogParam<bool>(ParamNames::ENABLE_FEATURE_TRACKER, false);
    declareAndLogParam<bool>(ParamNames::ENABLE_NN, false);
    declareAndLogParam<int>(ParamNames::MAX_Q_SIZE, 8);
    declareAndLogParam<bool>(ParamNames::LOW_BANDWIDTH, false);
    declareAndLogParam<int>(ParamNames::LOW_BANDWIDTH_PROFILE, 4);
    declareAndLogParam<int>(ParamNames::LOW_BANDWIDTH_FRAME_FREQ, 30);
    declareAndLogParam<int>(ParamNames::LOW_BANDWIDTH_BITRATE, 0);
    declareAndLogParam<int>(ParamNames::LOW_BANDWIDTH_QUALITY, 50);
    declareAndLogParam<std::string>(ParamNames::LOW_BANDWIDTH_FFMPEG_ENCODER, "libx264");
    declareAndLogParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP, false);
    declareAndLogParam<std::string>(ParamNames::CALIBRATION_FILE, "");
    declareAndLogParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG, false);
    declareAndLogParam<bool>(ParamNames::ENABLE_LAZY_PUBLISHER, true);
    declareAndLogParam<bool>(ParamNames::ADD_EXPOSURE_OFFSET, false);
    declareAndLogParam<int>(ParamNames::EXPOSURE_OFFSET, 0);
    declareAndLogParam<bool>(ParamNames::REVERSE_STEREO_SOCKET_ORDER, false);
    declareAndLogParam<bool>(ParamNames::SYNCED, false);
    declareAndLogParam<bool>(ParamNames::PUBLISH_COMPRESSED, false);
}
void SensorParamHandler::declareParams(std::shared_ptr<dai::node::Camera> cam, bool publish) {
    declareAndLogParam<bool>(ParamNames::PUBLISH_TOPIC, publish);
    declareAndLogParam<int>(ParamNames::WIDTH, 640);
    declareAndLogParam<int>(ParamNames::HEIGHT, 400);
    float fps = 30.0;
    declareAndLogParam<float>(ParamNames::FPS, fps);
    declareAndLogParam<bool>(ParamNames::UNDISTORTED, true);
    declareAndLogParam<std::string>(ParamNames::RESIZE_MODE, "CROP");

    size_t iso = declareAndLogParam(ParamNames::ISO, 800, getRangedIntDescriptor(100, 1600));
    size_t exposure = declareAndLogParam(ParamNames::EXPOSURE, 1000, getRangedIntDescriptor(1, 33000));

    if(declareAndLogParam<bool>(ParamNames::SET_MAN_EXPOSURE, false)) {
        cam->initialControl.setManualExposure(exposure, iso);
    }
    if(declareAndLogParam<bool>(ParamNames::FSYNC_CONTINUOUS, false)) {
        cam->initialControl.setFrameSyncMode(
            utils::getValFromMap(declareAndLogParam<std::string>(ParamNames::FSYNC_MODE, "INPUT"), dai_nodes::sensor_helpers::fSyncModeMap));
    }
    if(declareAndLogParam<bool>(ParamNames::FSYNC_TRIGGER, false)) {
        cam->initialControl.setExternalTrigger(declareAndLogParam<int>(ParamNames::NUM_FRAMES_BURST, 1),
                                               declareAndLogParam<int>(ParamNames::NUM_FRAMES_DISCARD, 0));
    }
    // not available yet
    // cam->initialControl->setImageOrientation(
    //     utils::getValFromMap(declareAndLogParam<std::string>(ParamNames::SENSOR_IMG_ORIENTATION, "AUTO"),
    //     dai_nodes::sensor_helpers::cameraImageOrientationMap));
    int expLimit = declareAndLogParam<int>(ParamNames::AUTO_EXPOSURE_LIMIT, 1000);
    if(declareAndLogParam<bool>(ParamNames::SET_AUTO_EXPOSURE_LIMIT, false)) {
        cam->initialControl.setAutoExposureLimit(expLimit);
    }
    if(declareAndLogParam(ParamNames::SET_MAN_FOCUS, false)) {
        cam->initialControl.setManualFocus(declareAndLogParam<int>(ParamNames::FOCUS, 1));
    }
    if(declareAndLogParam(ParamNames::SET_MAN_WHITEBALANCE, false)) {
        cam->initialControl.setManualWhiteBalance(declareAndLogParam<int>(ParamNames::WHITEBALANCE, 3000));
    }
    int sharpness = declareAndLogParam<int>(ParamNames::SHARPNESS, 1);
    if(declareAndLogParam(ParamNames::SET_SHARPNESS, false)) {
        cam->initialControl.setSharpness(sharpness);
    }
    int chromaDenoise = declareAndLogParam<int>(ParamNames::CHROMA_DENOISE, 1);
    if(declareAndLogParam(ParamNames::SET_CHROMA_DENOISE, false)) {
        cam->initialControl.setChromaDenoise(chromaDenoise);
    }
    int lumaDenoise = declareAndLogParam<int>(ParamNames::LUMA_DENOISE, 1);
    if(declareAndLogParam(ParamNames::SET_LUMA_DENOISE, false)) {
        cam->initialControl.setLumaDenoise(lumaDenoise);
    }
    bool setAutoExpRegion = declareAndLogParam<bool>(ParamNames::SET_AUTO_EXP_REGION, false);
    int autoExpStartX = declareAndLogParam<int>(ParamNames::AUTO_EXP_REGION_START_X, 0);
    int autoExpStartY = declareAndLogParam<int>(ParamNames::AUTO_EXP_REGION_START_Y, 0);
    int autoExpWidth = declareAndLogParam<int>(ParamNames::AUTO_EXP_REGION_WIDTH, 0);
    int autoExpHeight = declareAndLogParam<int>(ParamNames::AUTO_EXP_REGION_HEIGHT, 0);
    if(setAutoExpRegion) {
        cam->initialControl.setAutoExposureRegion(autoExpStartX, autoExpStartY, autoExpWidth, autoExpHeight);
    }
}
std::shared_ptr<dai::CameraControl> SensorParamHandler::setRuntimeParams(const std::vector<rclcpp::Parameter>& params) {
    auto ctrl = std::make_shared<dai::CameraControl>();
    for(const auto& p : params) {
        if(p.get_name() == getFullParamName(ParamNames::SET_MAN_EXPOSURE)) {
            if(p.get_value<bool>()) {
                ctrl->setManualExposure(getParam<int>(ParamNames::EXPOSURE), getParam<int>(ParamNames::ISO));
            } else {
                ctrl->setAutoExposureEnable();
            }
        } else if(p.get_name() == getFullParamName(ParamNames::EXPOSURE)) {
            if(getParam<bool>(ParamNames::SET_MAN_EXPOSURE)) {
                ctrl->setManualExposure(p.get_value<int>(), getParam<int>(ParamNames::ISO));
            }
        } else if(p.get_name() == getFullParamName(ParamNames::ISO)) {
            if(getParam<bool>(ParamNames::SET_MAN_EXPOSURE)) {
                ctrl->setManualExposure(getParam<int>(ParamNames::EXPOSURE), p.get_value<int>());
            }
        } else if(p.get_name() == getFullParamName(ParamNames::SET_MAN_FOCUS)) {
            if(p.get_value<bool>()) {
                ctrl->setManualFocus(getParam<int>(ParamNames::FOCUS));
            } else {
                ctrl->setAutoFocusMode(dai::CameraControl::AutoFocusMode::CONTINUOUS_PICTURE);
            }
        } else if(p.get_name() == getFullParamName(ParamNames::FOCUS)) {
            if(getParam<bool>(ParamNames::SET_MAN_FOCUS)) {
                ctrl->setManualFocus(p.get_value<int>());
            }
        } else if(p.get_name() == getFullParamName(ParamNames::SET_MAN_WHITEBALANCE)) {
            if(p.get_value<bool>()) {
                ctrl->setManualWhiteBalance(getParam<int>(ParamNames::WHITEBALANCE));
            } else {
                ctrl->setAutoWhiteBalanceMode(dai::CameraControl::AutoWhiteBalanceMode::AUTO);
            }
        } else if(p.get_name() == getFullParamName(ParamNames::WHITEBALANCE)) {
            if(getParam<bool>(ParamNames::SET_MAN_WHITEBALANCE)) {
                ctrl->setManualWhiteBalance(p.get_value<int>());
            }
        } else if(p.get_name() == getFullParamName(ParamNames::SET_AUTO_EXPOSURE_LIMIT)) {
            if(p.get_value<bool>()) {
                ctrl->setAutoExposureLimit(getParam<int>(ParamNames::AUTO_EXPOSURE_LIMIT));
            }
        } else if(p.get_name() == getFullParamName(ParamNames::AUTO_EXPOSURE_LIMIT)) {
            if(getParam<bool>(ParamNames::SET_AUTO_EXPOSURE_LIMIT)) {
                ctrl->setAutoExposureLimit(p.get_value<int>());
            }
        } else if(p.get_name() == getFullParamName(ParamNames::SET_SHARPNESS)) {
            if(p.get_value<bool>()) {
                ctrl->setSharpness(getParam<int>(ParamNames::SHARPNESS));
            }
        } else if(p.get_name() == getFullParamName(ParamNames::SHARPNESS)) {
            if(getParam<bool>(ParamNames::SET_SHARPNESS)) {
                ctrl->setSharpness(p.get_value<int>());
            }
        } else if(p.get_name() == getFullParamName(ParamNames::SET_CHROMA_DENOISE)) {
            if(p.get_value<bool>()) {
                ctrl->setChromaDenoise(getParam<int>(ParamNames::CHROMA_DENOISE));
            }
        } else if(p.get_name() == getFullParamName(ParamNames::CHROMA_DENOISE)) {
            if(getParam<bool>(ParamNames::SET_CHROMA_DENOISE)) {
                ctrl->setChromaDenoise(p.get_value<int>());
            }
        } else if(p.get_name() == getFullParamName(ParamNames::SET_LUMA_DENOISE)) {
            if(p.get_value<bool>()) {
                ctrl->setLumaDenoise(getParam<int>(ParamNames::LUMA_DENOISE));
            }
        } else if(p.get_name() == getFullParamName(ParamNames::LUMA_DENOISE)) {
            if(getParam<bool>(ParamNames::SET_LUMA_DENOISE)) {
                ctrl->setLumaDenoise(p.get_value<int>());
            }
        } else if(p.get_name() == getFullParamName(ParamNames::AUTO_EXP_REGION_START_X)) {
            if(getParam<bool>(ParamNames::SET_AUTO_EXP_REGION)) {
                ctrl->setAutoExposureRegion(p.get_value<int>(),
                                            getParam<int>(ParamNames::AUTO_EXP_REGION_START_Y),
                                            getParam<int>(ParamNames::AUTO_EXP_REGION_WIDTH),
                                            getParam<int>(ParamNames::AUTO_EXP_REGION_HEIGHT));
            }
        } else if(p.get_name() == getFullParamName(ParamNames::AUTO_EXP_REGION_START_Y)) {
            if(getParam<bool>(ParamNames::SET_AUTO_EXP_REGION)) {
                ctrl->setAutoExposureRegion(getParam<int>(ParamNames::AUTO_EXP_REGION_START_X),
                                            p.get_value<int>(),
                                            getParam<int>(ParamNames::AUTO_EXP_REGION_WIDTH),
                                            getParam<int>(ParamNames::AUTO_EXP_REGION_HEIGHT));
            }
        } else if(p.get_name() == getFullParamName(ParamNames::AUTO_EXP_REGION_WIDTH)) {
            if(getParam<bool>(ParamNames::SET_AUTO_EXP_REGION)) {
                ctrl->setAutoExposureRegion(getParam<int>(ParamNames::AUTO_EXP_REGION_START_X),
                                            getParam<int>(ParamNames::AUTO_EXP_REGION_START_Y),
                                            p.get_value<int>(),
                                            getParam<int>(ParamNames::AUTO_EXP_REGION_HEIGHT));
            }
        } else if(p.get_name() == getFullParamName(ParamNames::AUTO_EXP_REGION_HEIGHT)) {
            if(getParam<bool>(ParamNames::SET_AUTO_EXP_REGION)) {
                ctrl->setAutoExposureRegion(getParam<int>(ParamNames::AUTO_EXP_REGION_START_X),
                                            getParam<int>(ParamNames::AUTO_EXP_REGION_START_Y),
                                            getParam<int>(ParamNames::AUTO_EXP_REGION_WIDTH),
                                            p.get_value<int>());
            }
        }
    }
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver
