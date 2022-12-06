#include "depthai_ros_driver/dai_nodes/nn.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_ros_driver/dai_nodes/nn_wrappers/mobilenet.hpp"
#include "depthai_ros_driver/dai_nodes/nn_wrappers/segmentation.hpp"
#include "depthai_ros_driver/dai_nodes/nn_wrappers/yolo.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
NN::NN(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_INFO(node->get_logger(), "Creating node %s base", daiNodeName.c_str());
    ph = std::make_unique<param_handlers::NNParamHandler>(daiNodeName);
    auto family = ph->getNNFamily(getROSNode());
    switch(family) {
        case param_handlers::nn::NNFamily::Yolo: {
            nnNode = std::make_unique<dai_nodes::nn_wrappers::Yolo>(getName(), getROSNode(), pipeline);
            break;
        }
        case param_handlers::nn::NNFamily::Mobilenet: {
            nnNode = std::make_unique<dai_nodes::nn_wrappers::Mobilenet>(getName(), getROSNode(), pipeline);
            break;
        }
        case param_handlers::nn::NNFamily::Segmentation: {
            nnNode = std::make_unique<dai_nodes::nn_wrappers::Segmentation>(getName(), getROSNode(), pipeline);
            break;
        }
    }

    RCLCPP_INFO(node->get_logger(), "Base node %s created", daiNodeName.c_str());
};
void NN::setNames() {}

void NN::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {}

void NN::setupQueues(std::shared_ptr<dai::Device> device) {
    nnNode->setupQueues(device);
}
void NN::closeQueues() {
    nnNode->closeQueues();
}

void NN::link(const dai::Node::Input& in, int linkType) {
    nnNode->link(in, linkType);
}

dai::Node::Input NN::getInput(int linkType) {
    return nnNode->getInput(linkType);
}

void NN::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(getROSNode(), params);
    nnNode->updateParams(params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
