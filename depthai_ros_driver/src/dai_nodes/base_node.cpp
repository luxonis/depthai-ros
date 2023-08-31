#include "depthai_ros_driver/dai_nodes/base_node.hpp"

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
BaseNode::BaseNode(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> /*pipeline*/) {
    setNodeName(daiNodeName);
    setROSNodePointer(node);
    intraProcessEnabled = node->get_node_options().use_intra_process_comms();
};
BaseNode::~BaseNode() = default;
void BaseNode::setNodeName(const std::string& daiNodeName) {
    baseDAINodeName = daiNodeName;
};
void BaseNode::setROSNodePointer(rclcpp::Node* node) {
    baseNode = node;
};
rclcpp::Node* BaseNode::getROSNode() {
    return baseNode;
};
std::string BaseNode::getName() {
    return baseDAINodeName;
};

bool BaseNode::ipcEnabled() {
    return intraProcessEnabled;
}

std::string BaseNode::getTFPrefix(const std::string& frameName) {
    return std::string(getROSNode()->get_name()) + "_" + frameName;
}

dai::Node::Input BaseNode::getInput(int /*linkType = 0*/) {
    throw(std::runtime_error("getInput() not implemented"));
};

void BaseNode::closeQueues() {
    throw(std::runtime_error("closeQueues() not implemented"));
};

void BaseNode::setNames() {
    throw(std::runtime_error("setNames() not implemented"));
};

void BaseNode::setXinXout(std::shared_ptr<dai::Pipeline> /*pipeline*/) {
    throw(std::runtime_error("setXinXout() not implemented"));
};

void BaseNode::setupQueues(std::shared_ptr<dai::Device> /*device*/) {
    throw(std::runtime_error("setupQueues() not implemented"));
};

void BaseNode::link(dai::Node::Input /*in*/, int /*linkType = 0*/) {
    throw(std::runtime_error("link() not implemented"));
};

void BaseNode::updateParams(const std::vector<rclcpp::Parameter>& /*params*/) {
    return;
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver