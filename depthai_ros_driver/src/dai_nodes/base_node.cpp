#include "depthai_ros_driver/dai_nodes/base_node.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "ros/node_handle.h"

namespace depthai_ros_driver {
namespace dai_nodes {
BaseNode::BaseNode(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> /*pipeline*/) {
    setNodeName(daiNodeName);
    setROSNodePointer(node);
}
BaseNode::~BaseNode() = default;
void BaseNode::setNodeName(const std::string& daiNodeName) {
    baseDAINodeName = daiNodeName;
}
void BaseNode::setROSNodePointer(ros::NodeHandle node) {
    baseNode = node;
}
ros::NodeHandle BaseNode::getROSNode() {
    return baseNode;
}
std::string BaseNode::getName() {
    return baseDAINodeName;
}
std::string BaseNode::getTFPrefix(const std::string& frameName) {
    auto prefix = std::string(getROSNode().getNamespace()) + "_" + frameName;
    prefix.erase(0, 1);
    return prefix;
}
dai::Node::Input BaseNode::getInput(int /*linkType = 0*/) {
    throw(std::runtime_error("getInput() not implemented"));
}
}  // namespace dai_nodes
}  // namespace depthai_ros_driver