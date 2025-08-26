#include "depthai_ros_driver/dai_nodes/base_node.hpp"

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
BaseNode::BaseNode(
    const std::string& daiNodeName, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<dai::Pipeline> pipeline, std::string deviceName, bool rsCompat)
    : baseNode(node), pipeline(pipeline), deviceName(deviceName), baseDAINodeName(daiNodeName), rsCompat(rsCompat), logger(node->get_logger()) {
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

std::string BaseNode::getSocketName(dai::CameraBoardSocket socket) {
    return depthai_bridge::getSocketName(socket, deviceName, rsCompat);
}

std::string BaseNode::getDeviceName() {
    return deviceName;
}
bool BaseNode::rsCompatibilityMode() {
    return rsCompat;
}

std::string BaseNode::getFrameName(const std::string& frameName) {
    std::string prefix;
    if(getROSNode()->get_parameter("driver.i_publish_tf_from_calibration").as_bool()) {
        prefix = getROSNode()->get_parameter("driver.i_tf_base_frame").as_string();
    } else {
        prefix = getROSNode()->get_name();
    }

    return depthai_bridge::getFrameName(prefix, frameName);
}

std::string BaseNode::getOpticalFrameName(const std::string& frameName) {
    std::string prefix;
    if(getROSNode()->get_parameter("driver.i_publish_tf_from_calibration").as_bool()) {
        prefix = getROSNode()->get_parameter("driver.i_tf_base_frame").as_string();
    } else {
        prefix = getROSNode()->get_name();
    }

    return depthai_bridge::getOpticalFrameName(prefix, frameName, rsCompat);
}
dai::Node::Input& BaseNode::getInput(int /*linkType = 0*/) {
    throw(std::runtime_error("getInput() not implemented"));
};

dai::Node::Input& BaseNode::getInputByName(const std::string& /*name*/) {
    throw(std::runtime_error("getInputByName() not implemented"));
};

void BaseNode::closeQueues() {
    throw(std::runtime_error("closeQueues() not implemented"));
};

std::shared_ptr<sensor_helpers::ImagePublisher> BaseNode::setupOutput(
    std::shared_ptr<dai::Pipeline> pipeline, const std::string& qName, dai::Node::Output* out, bool isSynced, const utils::VideoEncoderConfig& encoderConfig) {
    return std::make_shared<sensor_helpers::ImagePublisher>(getROSNode(), pipeline, qName, out, isSynced, ipcEnabled(), encoderConfig);
};

void BaseNode::setNames() {
    throw(std::runtime_error("setNames() not implemented"));
};

void BaseNode::setInOut(std::shared_ptr<dai::Pipeline> /*pipeline*/) {
    throw(std::runtime_error("setInOut() not implemented"));
};

void BaseNode::setupQueues(std::shared_ptr<dai::Device> /*device*/) {
    throw(std::runtime_error("setupQueues() not implemented"));
};

void BaseNode::link(dai::Node::Input& /*in*/, int /*linkType = 0*/) {
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
