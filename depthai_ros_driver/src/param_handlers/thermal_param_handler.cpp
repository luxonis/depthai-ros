#include "depthai_ros_driver/param_handlers/thermal_param_handler.hpp"

#include <unordered_map>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/datatype/ThermalConfig.hpp"
#include "depthai/pipeline/node/Thermal.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
ThermalParamHandler::ThermalParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat)
    : BaseParamHandler(node, name, deviceName, rsCompat) {}
ThermalParamHandler::~ThermalParamHandler() = default;
void ThermalParamHandler::declareParams(std::shared_ptr<dai::node::Thermal> thermal) {
    thermalOrientMap = {{"NORMAL", dai::ThermalConfig::ThermalImageOrientation::Normal},
                        {"MIRROR", dai::ThermalConfig::ThermalImageOrientation::Mirror},
                        {"FLIP", dai::ThermalConfig::ThermalImageOrientation::Flip},
                        {"MIRROR_FLIP", dai::ThermalConfig::ThermalImageOrientation::MirrorFlip}};
    auto socketID = declareAndLogParam<int>(ParamNames::BOARD_SOCKET_ID, static_cast<int>(dai::CameraBoardSocket::CAM_E));
    thermal->build(static_cast<dai::CameraBoardSocket>(socketID));
    declareAndLogParam<int>(ParamNames::WIDTH, 256);
    declareAndLogParam<int>(ParamNames::HEIGHT, 162);
    declareAndLogParam<bool>(ParamNames::PUBLISH_TOPIC, true);
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
    declareAndLogParam<bool>(ParamNames::PUBLISH_RAW, true);
    auto thermalConfig = std::make_shared<dai::ThermalConfig>();
    thermalConfig->ffcParams.autoFFC = declareAndLogParam<bool>("r_auto_ffc", false);
    thermalConfig->ffcParams.closeManualShutter = declareAndLogParam<bool>("r_close_manual_shutter", false);
    thermalConfig->imageParams.brightnessLevel = declareAndLogParam<int>("r_brightness_level", 0, getRangedIntDescriptor(0, 255));
    thermalConfig->imageParams.orientation = utils::getValFromMap(declareAndLogParam<std::string>("i_orientation", "NORMAL"), thermalOrientMap);
    thermalConfig->imageParams.timeNoiseFilterLevel = declareAndLogParam<int>("r_time_noise_filter_level", 0, getRangedIntDescriptor(0, 3));
    declareAndLogParam<int>(ParamNames::FPS, 30);
}
std::shared_ptr<dai::ThermalConfig> ThermalParamHandler::setRuntimeParams(const std::vector<rclcpp::Parameter>& params) {
    auto conf = std::make_shared<dai::ThermalConfig>();
    for(const auto& p : params) {
        if(p.get_name() == getFullParamName("r_auto_ffc")) {
            conf->ffcParams.autoFFC = p.get_value<bool>();
        } else if(p.get_name() == getFullParamName("r_close_manual_shutter")) {
            conf->ffcParams.closeManualShutter = p.get_value<bool>();
        } else if(p.get_name() == getFullParamName("r_brightness_level")) {
            conf->imageParams.brightnessLevel = p.get_value<int>();
        } else if(p.get_name() == getFullParamName("i_orientation")) {
            conf->imageParams.orientation = utils::getValFromMap(p.get_value<std::string>(), thermalOrientMap);
        } else if(p.get_name() == getFullParamName("r_time_noise_filter_level")) {
            conf->imageParams.timeNoiseFilterLevel = p.get_value<int>();
        }
    }
    return conf;
}

}  // namespace param_handlers
}  // namespace depthai_ros_driver
