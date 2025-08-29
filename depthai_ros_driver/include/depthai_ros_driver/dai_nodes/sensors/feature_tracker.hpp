#pragma once

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_msgs/msg/tracked_features.hpp"
#include "rclcpp/publisher.hpp"

namespace dai {
class Pipeline;
class Device;
class MessageQueue;
class ADatatype;
namespace node {
class FeatureTracker;
}  // namespace node
}  // namespace dai

namespace depthai_bridge {
class TrackedFeaturesConverter;
}
namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
class FeatureTrackerParamHandler;
}
namespace dai_nodes {

class FeatureTracker : public BaseNode {
   public:
    explicit FeatureTracker(const std::string& daiNodeName,
                            std::shared_ptr<rclcpp::Node> node,
                            std::shared_ptr<dai::Pipeline> pipeline,
                            const std::string& deviceName,
                            bool rsCompat);
    ~FeatureTracker();
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input& in, int linkType = 0) override;
    dai::Node::Input& getInput(int linkType = 0) override;
    void setNames() override;
    void setInOut(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;
    void getParentName(const std::string& fullName);

   private:
    std::unique_ptr<depthai_bridge::TrackedFeaturesConverter> featureConverter;
    void featureQCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    rclcpp::Publisher<depthai_ros_msgs::msg::TrackedFeatures>::SharedPtr featurePub;
    std::shared_ptr<dai::node::FeatureTracker> featureNode;
    std::unique_ptr<param_handlers::FeatureTrackerParamHandler> ph;
    std::shared_ptr<dai::MessageQueue> featureQ;
    std::string featureQName;
    std::string parentName;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
