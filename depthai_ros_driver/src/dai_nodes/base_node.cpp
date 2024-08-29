#include "depthai_ros_driver/dai_nodes/base_node.hpp"

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
BaseNode::BaseNode(const std::string& daiNodeName, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<dai::Pipeline> /*pipeline*/)
    : baseNode(node), baseDAINodeName(daiNodeName), logger(node->get_logger()) {
    intraProcessEnabled = node->get_node_options().use_intra_process_comms();
};
BaseNode::~BaseNode() = default;
void BaseNode::setNodeName(const std::string& daiNodeName) {
    baseDAINodeName = daiNodeName;
};
void BaseNode::setROSNodePointer(std::shared_ptr<rclcpp::Node> node) {
    baseNode = node;
};
std::shared_ptr<rclcpp::Node> BaseNode::getROSNode() {
    return baseNode;
};
std::string BaseNode::getName() {
    return baseDAINodeName;
};

bool BaseNode::ipcEnabled() {
    return intraProcessEnabled;
}
rclcpp::Logger BaseNode::getLogger() {
    return logger;
}

bool BaseNode::rsCompabilityMode() {
    return sensor_helpers::rsCompabilityMode(getROSNode());
}

std::string BaseNode::getSocketName(dai::CameraBoardSocket socket) {
    return sensor_helpers::getSocketName(getROSNode(), socket);
}

std::string BaseNode::getTFPrefix(const std::string& frameName) {
    return std::string(getROSNode()->get_name()) + "_" + frameName;
}

std::string BaseNode::getOpticalTFPrefix(const std::string& frameName) {
    std::string suffix = "_camera_optical_frame";
    if(sensor_helpers::rsCompabilityMode(getROSNode())) {
        suffix = "_optical_frame";
    }
    return getTFPrefix(frameName) + suffix;
}
dai::Node::Input BaseNode::getInput(int /*linkType = 0*/) {
    throw(std::runtime_error("getInput() not implemented"));
};

dai::Node::Input BaseNode::getInputByName(const std::string& /*name*/) {
    throw(std::runtime_error("getInputByName() not implemented"));
};

void BaseNode::closeQueues() {
    throw(std::runtime_error("closeQueues() not implemented"));
};

std::shared_ptr<dai::node::XLinkOut> BaseNode::setupXout(std::shared_ptr<dai::Pipeline> pipeline, const std::string& name) {
    return utils::setupXout(pipeline, name);
};

std::shared_ptr<sensor_helpers::ImagePublisher> BaseNode::setupOutput(std::shared_ptr<dai::Pipeline> pipeline,
                                                                      const std::string& qName,
                                                                      std::function<void(dai::Node::Input input)> nodeLink,
                                                                      bool isSynced,
                                                                      const utils::VideoEncoderConfig& encoderConfig) {
    return std::make_shared<sensor_helpers::ImagePublisher>(getROSNode(), pipeline, qName, nodeLink, isSynced, ipcEnabled(), encoderConfig);
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

std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> BaseNode::getPublishers() {
    return std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>>();
};
void BaseNode::updateParams(const std::vector<rclcpp::Parameter>& /*params*/) {
    return;
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
