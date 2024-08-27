
#include "depthai_ros_driver/param_handlers/pipeline_gen_param_handler.hpp"

#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
PipelineGenParamHandler::PipelineGenParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name) : BaseParamHandler(node, name) {}
PipelineGenParamHandler::~PipelineGenParamHandler() = default;

void PipelineGenParamHandler::declareParams() {
    declareAndLogParam<bool>("i_enable_imu", true);
    declareAndLogParam<bool>("i_enable_diagnostics", true);
    declareAndLogParam<bool>("i_enable_sync", false);
}
dai::CameraControl PipelineGenParamHandler::setRuntimeParams(const std::vector<rclcpp::Parameter>& /*params*/) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver
