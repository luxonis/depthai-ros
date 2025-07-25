#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "depthai/common/CameraFeatures.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace dai {
namespace node {
class Camera;
}  // namespace node
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
class SensorParamHandler : public BaseParamHandler {
    struct ParamNames {
        static constexpr const char* LOW_BANDWIDTH = "i_low_bandwidth";
        static constexpr const char* LOW_BANDWIDTH_PROFILE = "i_low_bandwidth_profile";
        static constexpr const char* LOW_BANDWIDTH_FRAME_FREQ = "i_low_bandwidth_frame_freq";
        static constexpr const char* LOW_BANDWIDTH_BITRATE = "i_low_bandwidth_bitrate";
        static constexpr const char* LOW_BANDWIDTH_QUALITY = "i_low_bandwidth_quality";
        static constexpr const char* LOW_BANDWIDTH_FFMPEG_ENCODER = "i_low_bandwidth_ffmpeg_encoder";
        static constexpr const char* GET_BASE_DEVICE_TIMESTAMP = "i_get_base_device_timestamp";
        static constexpr const char* CALIBRATION_FILE = "i_calibration_file";
        static constexpr const char* UPDATE_ROS_BASE_TIME_ON_ROS_MSG = "i_update_ros_base_time_on_ros_msg";
        static constexpr const char* ENABLE_LAZY_PUBLISHER = "i_enable_lazy_publisher";
        static constexpr const char* PUBLISH_TOPIC = "i_publish_topic";
        static constexpr const char* ADD_EXPOSURE_OFFSET = "i_add_exposure_offset";
        static constexpr const char* EXPOSURE_OFFSET = "i_exposure_offset";
        static constexpr const char* REVERSE_STEREO_SOCKET_ORDER = "i_reverse_stereo_socket_order";
        static constexpr const char* SYNCED = "i_synced";
        static constexpr const char* PUBLISH_COMPRESSED = "i_publish_compressed";
        static constexpr const char* WIDTH = "i_width";
        static constexpr const char* HEIGHT = "i_height";
        static constexpr const char* PUBLISH_RAW = "i_publish_raw";
        static constexpr const char* ISO = "r_iso";
        static constexpr const char* EXPOSURE = "r_exposure";
        static constexpr const char* SET_MAN_EXPOSURE = "r_set_man_exposure";
        static constexpr const char* SET_MAN_FOCUS = "r_set_man_focus";
        static constexpr const char* FOCUS = "r_focus";
        static constexpr const char* SET_MAN_WHITEBALANCE = "r_set_man_whitebalance";
        static constexpr const char* WHITEBALANCE = "r_whitebalance";
        static constexpr const char* FSYNC_CONTINUOUS = "i_fsync_continuous";
        static constexpr const char* FSYNC_MODE = "i_fsync_mode";
        static constexpr const char* FSYNC_TRIGGER = "i_fsync_trigger";
        static constexpr const char* NUM_FRAMES_BURST = "i_num_frames_burst";
        static constexpr const char* NUM_FRAMES_DISCARD = "i_num_frames_discard";
        static constexpr const char* AUTO_EXPOSURE_LIMIT = "r_auto_exposure_limit";
        static constexpr const char* SET_AUTO_EXPOSURE_LIMIT = "r_set_auto_exposure_limit";
        static constexpr const char* SHARPNESS = "r_sharpness";
        static constexpr const char* SET_SHARPNESS = "r_set_sharpness";
        static constexpr const char* CHROMA_DENOISE = "r_chroma_denoise";
        static constexpr const char* SET_CHROMA_DENOISE = "r_set_chroma_denoise";
        static constexpr const char* LUMA_DENOISE = "r_luma_denoise";
        static constexpr const char* SET_LUMA_DENOISE = "r_set_luma_denoise";
        static constexpr const char* SET_AUTO_EXP_REGION = "r_set_auto_exp_region";
        static constexpr const char* AUTO_EXP_REGION_START_X = "r_auto_exp_region_start_x";
        static constexpr const char* AUTO_EXP_REGION_START_Y = "r_auto_exp_region_start_y";
        static constexpr const char* AUTO_EXP_REGION_WIDTH = "r_auto_exp_region_width";
        static constexpr const char* AUTO_EXP_REGION_HEIGHT = "r_auto_exp_region_height";

        // common params
        static constexpr const char* SIMULATE_FROM_TOPIC = "i_simulate_from_topic";
        static constexpr const char* SIMULATED_TOPIC_NAME = "i_simulated_topic_name";
        static constexpr const char* DISABLE_NODE = "i_disable_node";
        static constexpr const char* BOARD_SOCKET_ID = "i_board_socket_id";
        static constexpr const char* ENABLE_FEATURE_TRACKER = "i_enable_feature_tracker";
        static constexpr const char* ENABLE_NN = "i_enable_nn";
    };

   public:
    explicit SensorParamHandler(
        std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat, dai::CameraBoardSocket socket);
    ~SensorParamHandler();
    void declareCommonParams(dai::CameraBoardSocket socket);
    void declareParams(std::shared_ptr<dai::node::Camera> cam, dai::CameraFeatures, bool publish);
    std::shared_ptr<dai::CameraControl> setRuntimeParams(const std::vector<rclcpp::Parameter>& params) override;

   private:
    dai::CameraBoardSocket socketID;
    std::unordered_map<dai::ImgFrame::Type, std::string> frameTypeMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
