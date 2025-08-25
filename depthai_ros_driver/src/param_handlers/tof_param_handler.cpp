#include "depthai_ros_driver/param_handlers/tof_param_handler.hpp"

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "depthai/pipeline/node/ToF.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
ToFParamHandler::ToFParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat)
    : BaseParamHandler(node, name, deviceName, rsCompat) {
    medianFilterMap = {{"MEDIAN_OFF", dai::MedianFilter::MEDIAN_OFF},
                       {"KERNEL_3x3", dai::MedianFilter::KERNEL_3x3},
                       {"KERNEL_5x5", dai::MedianFilter::KERNEL_5x5},
                       {"KERNEL_7x7", dai::MedianFilter::KERNEL_7x7}};
    presetModeMap = {{"TOF_LOW_RANGE", dai::ImageFiltersPresetMode::TOF_LOW_RANGE},
                     {"TOF_MID_RANGE", dai::ImageFiltersPresetMode::TOF_MID_RANGE},
                     {"TOF_HIGH_RANGE", dai::ImageFiltersPresetMode::TOF_HIGH_RANGE}};
}
ToFParamHandler::~ToFParamHandler() = default;
void ToFParamHandler::declareParams(std::shared_ptr<dai::node::ToF> tof, dai::CameraBoardSocket socket) {
    declareAndLogParam<bool>(ParamNames::PUBLISH_TOPIC, true);
    declareAndLogParam<bool>(ParamNames::SYNCED, false);
    declareAndLogParam<bool>(ParamNames::LOW_BANDWIDTH, false);
    declareAndLogParam<int>(ParamNames::LOW_BANDWIDTH_PROFILE, 4);
    declareAndLogParam<int>(ParamNames::LOW_BANDWIDTH_BITRATE, 0);
    declareAndLogParam<int>(ParamNames::LOW_BANDWIDTH_FRAME_FREQ, 30);
    declareAndLogParam<int>(ParamNames::LOW_BANDWIDTH_QUALITY, 80);
    declareAndLogParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP, false);
    declareAndLogParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG, false);
    declareAndLogParam<bool>(ParamNames::ADD_EXPOSURE_OFFSET, false);
    declareAndLogParam<int>(ParamNames::EXPOSURE_OFFSET, 0);
    declareAndLogParam<bool>(ParamNames::ENABLE_LAZY_PUBLISHER, false);
    declareAndLogParam<bool>(ParamNames::REVERSE_STEREO_SOCKET_ORDER, false);
    declareAndLogParam<std::string>(ParamNames::CALIBRATION_FILE, "");
    declareAndLogParam<int>(ParamNames::MAX_Q_SIZE, 8);
    declareAndLogParam<int>(ParamNames::WIDTH, 640);
    declareAndLogParam<int>(ParamNames::HEIGHT, 400);
    declareAndLogParam<bool>(ParamNames::ALIGNED, false);
    declareAndLogParam<bool>("i_run_align_on_host", true);
    auto fps = declareAndLogParam<float>(ParamNames::FPS, 30.0);

    auto sock = static_cast<dai::CameraBoardSocket>(declareAndLogParam<int>(ParamNames::BOARD_SOCKET_ID, static_cast<int>(socket)));
    dai::ImageFiltersPresetMode presetMode = utils::getValFromMap(declareAndLogParam<std::string>("i_preset_mode", "TOF_MID_RANGE"), presetModeMap);
    tof->build(sock, presetMode, fps);
}

}  // namespace param_handlers
}  // namespace depthai_ros_driver
