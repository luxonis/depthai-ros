#include "depthai_ros_driver/param_handlers/rgbd_param_handler.hpp"

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/node/host/RGBD.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
RGBDParamHandler::RGBDParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat)
    : BaseParamHandler(node, name, deviceName, rsCompat) {}
RGBDParamHandler::~RGBDParamHandler() = default;
void RGBDParamHandler::declareParams(std::shared_ptr<dai::node::RGBD> rgbd, dai::CameraBoardSocket socket) {
    declareAndLogParam<bool>(ParamNames::PUBLISH_TOPIC, true);
    declareAndLogParam<int>(ParamNames::BOARD_SOCKET_ID, static_cast<int>(socket));
    declareAndLogParam<int>(ParamNames::MAX_Q_SIZE, 2);
    declareAndLogParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP, false);
    declareAndLogParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG, false);
    int threadNum = declareAndLogParam<int>("i_num_threads", 1);
    if(threadNum > 1) {
        rgbd->useCPUMT(threadNum);
    }
    rgbd->runSyncOnHost(declareAndLogParam<bool>("i_run_sync_on_host", true));
    declareAndLogParam<bool>("i_run_align_on_host", true);
}

}  // namespace param_handlers
}  // namespace depthai_ros_driver
