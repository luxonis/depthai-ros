#include "depthai_ros_driver/dai_nodes/sensors/feature_tracker.hpp"

#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/FeatureTracker.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/TrackedFeaturesConverter.hpp"
#include "depthai_ros_driver/param_handlers/feature_tracker_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "depthai_ros_msgs/msg/tracked_features.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
FeatureTracker::FeatureTracker(const std::string& daiNodeName, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<dai::Pipeline> pipeline)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
    getParentName(daiNodeName);
    setNames();
    featureNode = pipeline->create<dai::node::FeatureTracker>();
    ph = std::make_unique<param_handlers::FeatureTrackerParamHandler>(node, daiNodeName);
    ph->declareParams(featureNode);
    setXinXout(pipeline);
    RCLCPP_DEBUG(getLogger(), "Node %s created", daiNodeName.c_str());
}
FeatureTracker::~FeatureTracker() = default;

void FeatureTracker::getParentName(const std::string& fullName) {
    auto endIdx = fullName.find("_");
    parentName = fullName.substr(0, endIdx);
}

void FeatureTracker::setNames() {
    featureQName = parentName + getName() + "_queue";
}

void FeatureTracker::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutFeature = pipeline->create<dai::node::XLinkOut>();
    xoutFeature->setStreamName(featureQName);
    featureNode->outputFeatures.link(xoutFeature->input);
}

void FeatureTracker::setupQueues(std::shared_ptr<dai::Device> device) {
    featureQ = device->getOutputQueue(featureQName, ph->getParam<int>("i_max_q_size"), false);
    auto tfPrefix = getTFPrefix(parentName);
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = rclcpp::QosOverridingOptions();
    featureConverter = std::make_unique<dai::ros::TrackedFeaturesConverter>(tfPrefix + "_frame", ph->getParam<bool>("i_get_base_device_timestamp"));
    featureConverter->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));

    featurePub = getROSNode()->create_publisher<depthai_ros_msgs::msg::TrackedFeatures>("~/" + getName() + "/tracked_features", 10, options);
    featureQ->addCallback(std::bind(&FeatureTracker::featureQCB, this, std::placeholders::_1, std::placeholders::_2));
}

void FeatureTracker::closeQueues() {
    featureQ->close();
}

void FeatureTracker::featureQCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto featureData = std::dynamic_pointer_cast<dai::TrackedFeatures>(data);
    std::deque<depthai_ros_msgs::msg::TrackedFeatures> deq;
    featureConverter->toRosMsg(featureData, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        featurePub->publish(currMsg);
        deq.pop_front();
    }
}

void FeatureTracker::link(dai::Node::Input in, int /*linkType*/) {
    featureNode->outputFeatures.link(in);
}

dai::Node::Input FeatureTracker::getInput(int /*linkType*/) {
    return featureNode->inputImage;
}

void FeatureTracker::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
