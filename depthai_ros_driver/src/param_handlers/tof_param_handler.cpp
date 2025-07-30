#include "depthai_ros_driver/param_handlers/tof_param_handler.hpp"

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "depthai/pipeline/node/ToF.hpp"
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
    declareAndLogParam<bool>("i_publish_topic", true);
    declareAndLogParam<bool>("i_synced", false);
    declareAndLogParam<bool>("i_low_bandwidth", false);
    declareAndLogParam<int>("i_low_bandwidth_profile", 4);
    declareAndLogParam<int>("i_low_bandwidth_bitrate", 0);
    declareAndLogParam<int>("i_low_bandwidth_frame_freq", 30);
    declareAndLogParam<int>("i_low_bandwidth_quality", 80);
    declareAndLogParam<bool>("i_get_base_device_timestamp", false);
    declareAndLogParam<bool>("i_update_ros_base_time_on_ros_msg", false);
    declareAndLogParam<bool>("i_add_exposure_offset", false);
    declareAndLogParam<int>("i_exposure_offset", 0);
    declareAndLogParam<bool>("i_enable_lazy_publisher", false);
    declareAndLogParam<bool>("i_reverse_stereo_socket_order", false);
    declareAndLogParam<std::string>("i_calibration_file", "");
    declareAndLogParam<int>("i_max_q_size", 8);
    declareAndLogParam<int>("i_width", 640);
    declareAndLogParam<int>("i_height", 400);
    auto fps = declareAndLogParam<float>("i_fps", 15.0);

    auto sock = static_cast<dai::CameraBoardSocket>(declareAndLogParam<int>("i_board_socket_id", static_cast<int>(socket)));
    dai::ImageFiltersPresetMode presetMode = utils::getValFromMap(declareAndLogParam<std::string>("i_preset_mode", "TOF_MID_RANGE"), presetModeMap);
    tof->build(sock, presetMode, fps);
}

}  // namespace param_handlers
}  // namespace depthai_ros_driver
