#pragma once
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/node.hpp"
namespace depthai_ros_driver {
namespace param_handlers {
struct ParamNames {
    static constexpr const char* ALIGNED = "i_aligned";
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
    static constexpr const char* MAX_Q_SIZE = "i_max_q_size";

    static constexpr const char* ADD_EXPOSURE_OFFSET = "i_add_exposure_offset";
    static constexpr const char* EXPOSURE_OFFSET = "i_exposure_offset";
    static constexpr const char* REVERSE_STEREO_SOCKET_ORDER = "i_reverse_stereo_socket_order";
    static constexpr const char* SYNCED = "i_synced";
    static constexpr const char* PUBLISH_COMPRESSED = "i_publish_compressed";
    static constexpr const char* PUBLISH_RAW = "i_publish_raw";
    static constexpr const char* WIDTH = "i_width";
    static constexpr const char* HEIGHT = "i_height";
    static constexpr const char* FPS = "i_fps";
    static constexpr const char* RESIZE_MODE = "i_resize_mode";
    static constexpr const char* UNDISTORTED = "i_undistorted";
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
inline rcl_interfaces::msg::ParameterDescriptor getRangedIntDescriptor(uint16_t min, uint16_t max) {
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.integer_range.resize(1);
        desc.integer_range.at(0).from_value = min;
        desc.integer_range.at(0).to_value = max;
        return desc;
    }
}
class BaseParamHandler {
   public:
    BaseParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat)
        : baseName(name), deviceName(deviceName), rsCompat(rsCompat), baseNode(node){};
    virtual ~BaseParamHandler() = default;
    virtual std::shared_ptr<dai::CameraControl> setRuntimeParams(const std::vector<rclcpp::Parameter>& /* params */) {
        return std::make_shared<dai::CameraControl>();
    }
    std::string getName() {
        return baseName;
    }
    dai::CameraBoardSocket getSocketID() {
        return static_cast<dai::CameraBoardSocket>(getParam<int>(ParamNames::BOARD_SOCKET_ID));
    }
    template <typename T>
    T getParam(const std::string& paramName) {
        T value;
        if(!baseNode->has_parameter(getFullParamName(paramName))) {
            RCLCPP_WARN(baseNode->get_logger(), "Parameter %s not found", getFullParamName(paramName).c_str());
        }
        baseNode->get_parameter<T>(getFullParamName(paramName), value);
        return value;
    }
    template <typename T>
    T getOtherNodeParam(const std::string& daiNodeName, const std::string& paramName) {
        T value;
        if(!baseNode->has_parameter(getFullParamName(daiNodeName, paramName))) {
            RCLCPP_WARN(baseNode->get_logger(), "Parameter %s not found", getFullParamName(daiNodeName, paramName).c_str());
        }
        baseNode->get_parameter<T>(getFullParamName(daiNodeName, paramName), value);
        return value;
    }

    std::string getFullParamName(const std::string& paramName) {
        return baseName + "." + paramName;
    }
    std::string getFullParamName(const std::string& daiNodeName, const std::string& paramName) {
        std::string name = daiNodeName + "." + paramName;
        return name;
    }

   protected:
    std::shared_ptr<rclcpp::Node> getROSNode() {
        return baseNode;
    }
    std::string getSocketName(dai::CameraBoardSocket socket) {
        return depthai_bridge::getSocketName(socket, deviceName, rsCompat);
    }
    template <typename T>
    T declareAndLogParam(const std::string& paramName, const std::vector<T>& value, bool override = false) {
        std::string fullName = baseName + "." + paramName;
        if(baseNode->has_parameter(fullName)) {
            if(override) {
                auto param = rclcpp::Parameter(fullName, value);
                baseNode->set_parameter(param);
            }
            return getParam<T>(paramName);
        } else {
            auto val = baseNode->declare_parameter<T>(fullName, value);
            logParam(fullName, val);
            return val;
        }
    }

    template <typename T>
    T declareAndLogParam(const std::string& paramName, T value, bool override = false) {
        std::string fullName = baseName + "." + paramName;
        if(baseNode->has_parameter(fullName)) {
            if(override) {
                auto param = rclcpp::Parameter(fullName, value);
                baseNode->set_parameter(param);
            }
            return getParam<T>(paramName);
        } else {
            auto val = baseNode->declare_parameter<T>(fullName, value);
            logParam(fullName, val);
            return val;
        }
    }
    template <typename T>
    T declareAndLogParam(const std::string& paramName, T value, rcl_interfaces::msg::ParameterDescriptor descriptor, bool override = false) {
        std::string fullName = baseName + "." + paramName;
        if(baseNode->has_parameter(fullName)) {
            if(override) {
                auto param = rclcpp::Parameter(fullName, value);
                baseNode->set_parameter(param);
            }
            return getParam<T>(fullName);
        } else {
            auto val = baseNode->declare_parameter<T>(fullName, value, descriptor);
            logParam(fullName, val);
            return val;
        }
    }
    template <typename T>
    T declareAndLogParam(const std::string& paramName,
                         const std::vector<T>& value,
                         rcl_interfaces::msg::ParameterDescriptor descriptor,
                         bool override = false) {
        std::string fullName = baseName + "." + paramName;
        if(baseNode->has_parameter(fullName)) {
            if(override) {
                auto param = rclcpp::Parameter(fullName, value);
                baseNode->set_parameter(param);
            }
            return getParam<T>(paramName);
        } else {
            auto val = baseNode->declare_parameter<T>(fullName, value, descriptor);
            logParam(fullName, val);
            return val;
        }
    }
    template <typename T>
    inline void logParam(const std::string& name, T value) {
        std::stringstream ss;
        ss << value;
        RCLCPP_DEBUG(baseNode->get_logger(), "Setting param %s with value %s", name.c_str(), ss.str().c_str());
    }
    template <typename T>
    inline void logParam(const std::string& name, const std::vector<T>& value) {
        std::stringstream ss;
        for(const auto& v : value) {
            ss << v << " ";
        }
        RCLCPP_DEBUG(baseNode->get_logger(), "Setting param %s with value %s", name.c_str(), ss.str().c_str());
    }
    std::string baseName;
    std::string deviceName;
    bool rsCompat;
    std::shared_ptr<rclcpp::Node> baseNode;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
