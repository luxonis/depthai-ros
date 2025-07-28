#include "depthai_ros_driver/param_handlers/rgbd_param_handler.hpp"

#include "depthai/pipeline/node/host/RGBD.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
RGBDParamHandler::RGBDParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat)
    : BaseParamHandler(node, name, deviceName, rsCompat) {}
RGBDParamHandler::~RGBDParamHandler() = default;
void RGBDParamHandler::declareParams(std::shared_ptr<dai::node::RGBD> rgbd) {
    declareAndLogParam<bool>("i_publish_topic", true);
    declareAndLogParam<int>("i_board_socket_id", static_cast<int>(dai::CameraBoardSocket::CAM_A));
}

}  // namespace param_handlers
}  // namespace depthai_ros_driver
