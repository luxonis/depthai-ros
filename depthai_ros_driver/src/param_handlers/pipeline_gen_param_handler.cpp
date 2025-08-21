
#include "depthai_ros_driver/param_handlers/pipeline_gen_param_handler.hpp"

#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
PipelineGenParamHandler::PipelineGenParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat)
    : BaseParamHandler(node, name, deviceName, rsCompat) {}
PipelineGenParamHandler::~PipelineGenParamHandler() = default;

void PipelineGenParamHandler::declareParams() {
    declareAndLogParam<std::string>("i_pipeline_type", "RGBD");
    declareAndLogParam<std::string>("i_nn_type", "spatial");
    declareAndLogParam<bool>("i_enable_imu", true);
    declareAndLogParam<bool>("i_enable_diagnostics", false);
    declareAndLogParam<bool>("i_enable_rgbd", false);
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver
