#include "depthai_ros_driver/param_handlers/sync_param_handler.hpp"

#include "depthai/pipeline/node/Sync.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
SyncParamHandler::SyncParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name) : BaseParamHandler(node, name) {}
SyncParamHandler::~SyncParamHandler() = default;
void SyncParamHandler::declareParams(std::shared_ptr<dai::node::Sync> sync) {
    sync->setSyncThreshold(std::chrono::milliseconds(declareAndLogParam<int>("sync_threshold", 10)));
    sync->setSyncAttempts(declareAndLogParam<int>("sync_attempts", 10));
}

dai::CameraControl SyncParamHandler::setRuntimeParams(const std::vector<rclcpp::Parameter>& /*params*/) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver
