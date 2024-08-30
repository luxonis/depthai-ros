
#include "depthai_ros_driver/param_handlers/pipeline_gen_param_handler.hpp"

#include "ros/node_handle.h"

namespace depthai_ros_driver {
namespace param_handlers {
PipelineGenParamHandler::PipelineGenParamHandler(ros::NodeHandle node, const std::string& name) : BaseParamHandler(node, name) {}
PipelineGenParamHandler::~PipelineGenParamHandler() = default;

void PipelineGenParamHandler::declareParams() {
    declareAndLogParam<bool>("i_enable_imu", true);
    declareAndLogParam<bool>("i_enable_diagnostics", true);
    declareAndLogParam<bool>("i_enable_sync", false);
}
dai::CameraControl PipelineGenParamHandler::setRuntimeParams(parametersConfig& config) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver
