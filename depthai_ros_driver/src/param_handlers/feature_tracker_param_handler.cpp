#include "depthai_ros_driver/param_handlers/feature_tracker_param_handler.hpp"

#include "depthai/pipeline/node/FeatureTracker.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
FeatureTrackerParamHandler::FeatureTrackerParamHandler(rclcpp::Node* node, const std::string& name) : BaseParamHandler(node, name) {}
FeatureTrackerParamHandler::~FeatureTrackerParamHandler() = default;
void FeatureTrackerParamHandler::declareParams(std::shared_ptr<dai::node::FeatureTracker> featureTracker) {
   
    declareAndLogParam<bool>("i_get_base_device_timestamp", false);

    featureTracker->setHardwareResources(declareAndLogParam<int>("i_num_shaves", 2), declareAndLogParam<int>("i_num_memory_slices", 2));

    auto config = featureTracker->initialConfig.get();
    // config.motionEstimator.type = 
}

dai::CameraControl FeatureTrackerParamHandler::setRuntimeParams(const std::vector<rclcpp::Parameter>& /*params*/) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver