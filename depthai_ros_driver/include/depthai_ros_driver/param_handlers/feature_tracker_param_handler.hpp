#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/datatype/FeatureTrackerConfig.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace dai {
namespace node {
class FeatureTracker;
}
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {

class FeatureTrackerParamHandler : public BaseParamHandler {
   public:
    explicit FeatureTrackerParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name);
    ~FeatureTrackerParamHandler();
    void declareParams(std::shared_ptr<dai::node::FeatureTracker> featureTracker);
    dai::CameraControl setRuntimeParams(const std::vector<rclcpp::Parameter>& params) override;
    std::unordered_map<std::string, dai::FeatureTrackerConfig::MotionEstimator::Type> motionEstMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
