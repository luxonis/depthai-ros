#include "depthai_ros_driver/param_handlers/feature_tracker_param_handler.hpp"

#include "depthai/pipeline/node/FeatureTracker.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
FeatureTrackerParamHandler::FeatureTrackerParamHandler(std::shared_ptr<rclcpp::Node> node,
                                                       const std::string& name,
                                                       const std::string& deviceName,
                                                       bool rsCompat)
    : BaseParamHandler(node, name, deviceName, rsCompat) {}
FeatureTrackerParamHandler::~FeatureTrackerParamHandler() = default;
void FeatureTrackerParamHandler::declareParams(std::shared_ptr<dai::node::FeatureTracker> featureTracker) {
    declareAndLogParam<bool>("i_get_base_device_timestamp", false);

    featureTracker->setHardwareResources(declareAndLogParam<int>("i_num_shaves", 2), declareAndLogParam<int>("i_num_memory_slices", 2));
    motionEstMap = {{"LUCAS_KANADE_OPTICAL_FLOW", dai::FeatureTrackerConfig::MotionEstimator::Type::LUCAS_KANADE_OPTICAL_FLOW},
                    {"HW_MOTION_ESTIMATION", dai::FeatureTrackerConfig::MotionEstimator::Type::HW_MOTION_ESTIMATION}

    };
    auto config = featureTracker->initialConfig;
    config->motionEstimator.type = (motionEstMap.at(declareAndLogParam<std::string>("i_motion_estimator", "LUCAS_KANADE_OPTICAL_FLOW")));
    featureTracker->initialConfig = config;
}

}  // namespace param_handlers
}  // namespace depthai_ros_driver
