#include "depthai_ros_driver/dai_nodes/base_node.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
BaseNode::BaseNode(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> /*pipeline*/) {
    setNodeName(daiNodeName);
    setROSNodePointer(node);
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
std::string BaseNode::getTFPrefix(const std::string& frameName) {
    return std::string(getROSNode()->get_name()) + "_" + frameName;
}
std::string BaseNode::getFrameNameFromSocket(dai::CameraBoardSocket socket, std::vector<dai::CameraFeatures> camFeatures) {
    std::string name;
    for(auto& cam : camFeatures) {
        if(cam.socket == socket) {
            if(cam.name == "color"){
                name = "rgb";
            }
            else{
                name = cam.name;
            }
            return getTFPrefix(name);
        }
    }
    throw std::runtime_error("Camera socket not found");
}
dai::Node::Input BaseNode::getInput(int /*linkType = 0*/) {
    throw(std::runtime_error("getInput() not implemented"));
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver