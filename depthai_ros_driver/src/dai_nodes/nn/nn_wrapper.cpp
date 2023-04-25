#include "depthai_ros_driver/dai_nodes/nn/nn_wrapper.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai_ros_driver/dai_nodes/nn/detection.hpp"
#include "depthai_ros_driver/dai_nodes/nn/segmentation.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
NNWrapper::NNWrapper(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s base", daiNodeName.c_str());
    ph = std::make_unique<param_handlers::NNParamHandler>(node, daiNodeName);
    auto family = ph->getNNFamily();
    switch(family) {
        case param_handlers::nn::NNFamily::Yolo: {
            nnNode = std::make_unique<dai_nodes::nn::Detection<dai::node::YoloDetectionNetwork>>(getName(), getROSNode(), pipeline);
            break;
        }
        case param_handlers::nn::NNFamily::Mobilenet: {
            nnNode = std::make_unique<dai_nodes::nn::Detection<dai::node::MobileNetDetectionNetwork>>(getName(), getROSNode(), pipeline);
            break;
        }
        case param_handlers::nn::NNFamily::Segmentation: {
            nnNode = std::make_unique<dai_nodes::nn::Segmentation>(getName(), getROSNode(), pipeline);
            break;
        }
    }

    RCLCPP_DEBUG(node->get_logger(), "Base node %s created", daiNodeName.c_str());
}
NNWrapper::~NNWrapper() = default;

void NNWrapper::setNames() {}

void NNWrapper::setXinXout(std::shared_ptr<dai::Pipeline> /*pipeline*/) {}

void NNWrapper::setupQueues(std::shared_ptr<dai::Device> device) {
    nnNode->setupQueues(device);
}
void NNWrapper::closeQueues() {
    nnNode->closeQueues();
}

void NNWrapper::link(dai::Node::Input in, int linkType) {
    nnNode->link(in, linkType);
}

dai::Node::Input NNWrapper::getInput(int linkType) {
    return nnNode->getInput(linkType);
}

void NNWrapper::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(params);
    nnNode->updateParams(params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
