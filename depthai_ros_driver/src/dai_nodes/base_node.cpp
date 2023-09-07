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
dai::Node::Input BaseNode::getInput(int /*linkType*/) {
    throw(std::runtime_error("getInput() not implemented"));
}
void BaseNode::closeQueues() {
    throw(std::runtime_error("closeQueues() not implemented"));
}

void BaseNode::setNames() {
    throw(std::runtime_error("setNames() not implemented"));
}

void BaseNode::setXinXout(std::shared_ptr<dai::Pipeline> /*pipeline*/) {
    throw(std::runtime_error("setXinXout() not implemented"));
}

void BaseNode::setupQueues(std::shared_ptr<dai::Device> /*device*/) {
    throw(std::runtime_error("setupQueues() not implemented"));
}

void BaseNode::link(dai::Node::Input /*in*/, int /*linkType = 0*/) {
    throw(std::runtime_error("link() not implemented"));
}

void BaseNode::updateParams(parametersConfig& /*config*/) {
    return;
}
}  // namespace dai_nodes
}  // namespace depthai_ros_driver