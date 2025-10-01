#include "depthai_ros_driver/param_handlers/vio_param_handler.hpp"

#include "depthai/basalt/BasaltVIO.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
VioParamHandler::VioParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat)
    : BaseParamHandler(node, name, deviceName, rsCompat) {}
VioParamHandler::~VioParamHandler() = default;
void VioParamHandler::declareParams(std::shared_ptr<dai::node::BasaltVIO> vio) {
    declareAndLogParam<bool>(ParamNames::PUBLISH_TOPIC, true);
    declareAndLogParam<int>(ParamNames::MAX_Q_SIZE, 2);
    declareAndLogParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP, false);
    declareAndLogParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG, false);
    declareAndLogParam<int>(ParamNames::BOARD_SOCKET_ID, static_cast<int>(dai::CameraBoardSocket::CAM_B));
    declareAndLogParam<int>("i_imu_update_rate", 400);
    declareAndLogParam<int>(ParamNames::WIDTH, 640);
    declareAndLogParam<int>(ParamNames::HEIGHT, 400);
    declareAndLogParam<double>(ParamNames::FPS, 60.0);
    declareAndLogParam<std::string>("i_vio_config_path", "");
    declareAndLogParam<bool>("i_publish_tf", true);
    declareAndLogParam<std::string>("i_frame_id", "odom");
    declareAndLogParam<std::string>("i_child_frame_id", "oak_parent_frame");
    std::string configPath = declareAndLogParam<std::string>("i_config_path", "");
    if(!configPath.empty()) {
        vio->setConfigPath(configPath);
    }
    declareAndLogParam<std::vector<double>>("i_covariance", std::vector<double>(36, 0.0));
}

}  // namespace param_handlers
}  // namespace depthai_ros_driver
