#include "depthai_ros_driver/dai_nodes/nn/nn_wrapper.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai_ros_driver/dai_nodes/nn/detection.hpp"
#include "depthai_ros_driver/dai_nodes/nn/segmentation.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "ros/node_handle.h"

namespace depthai_ros_driver {
namespace dai_nodes {
NNWrapper::NNWrapper(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> pipeline, const dai::CameraBoardSocket& socket)
    : BaseNode(daiNodeName, node, pipeline) {
    ROS_DEBUG("Creating node %s base", daiNodeName.c_str());
    ph = std::make_unique<param_handlers::NNParamHandler>(node, daiNodeName, socket);
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

    ROS_DEBUG("Base node %s created", daiNodeName.c_str());
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

void NNWrapper::updateParams(parametersConfig& config) {
    ph->setRuntimeParams(config);
    nnNode->updateParams(config);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
