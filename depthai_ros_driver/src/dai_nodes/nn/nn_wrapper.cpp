#include "depthai_ros_driver/dai_nodes/nn/nn_wrapper.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai_ros_driver/dai_nodes/nn/detection.hpp"
#include "depthai_ros_driver/dai_nodes/nn/segmentation.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
NNWrapper::NNWrapper(const std::string& daiNodeName,
                     std::shared_ptr<rclcpp::Node> node,
                     std::shared_ptr<dai::Pipeline> pipeline,
                     const std::string& deviceName,
                     bool rsCompat,
                     SensorWrapper& camNode,
                     const dai::CameraBoardSocket& socket)
    : BaseNode(daiNodeName, node, pipeline, deviceName, rsCompat) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s base", daiNodeName.c_str());
    ph = std::make_unique<param_handlers::NNParamHandler>(node, daiNodeName, deviceName, rsCompat, socket);
    auto family = ph->getNNFamily();
    switch(family) {
        case param_handlers::nn::NNFamily::Detection: {
            nnNode = std::make_unique<dai_nodes::nn::Detection>(getName(), getROSNode(), pipeline, deviceName, rsCompat, camNode, socket);
            break;
        }
        case param_handlers::nn::NNFamily::Segmentation: {
            RCLCPP_WARN(node->get_logger(), "Warning: Segmentation is not supported yet on RVC4");
            nnNode = std::make_unique<dai_nodes::nn::Segmentation>(getName(), getROSNode(), pipeline, deviceName, rsCompat, camNode, socket);
            break;
        }
        default:
            RCLCPP_ERROR(node->get_logger(), "NN family %d not supported", static_cast<int>(family));
            break;
    }

    RCLCPP_DEBUG(node->get_logger(), "Base node %s created", daiNodeName.c_str());
}
NNWrapper::~NNWrapper() = default;

void NNWrapper::setNames() {}

void NNWrapper::setInOut(std::shared_ptr<dai::Pipeline> /*pipeline*/) {}

void NNWrapper::setupQueues(std::shared_ptr<dai::Device> device) {
    nnNode->setupQueues(device);
}
void NNWrapper::closeQueues() {
    nnNode->closeQueues();
}

void NNWrapper::link(dai::Node::Input& in, int linkType) {
    nnNode->link(in, linkType);
}

dai::Node::Input& NNWrapper::getInput(int linkType) {
    return nnNode->getInput(linkType);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
