#include "depthai_ros_driver/dai_nodes/nn/nn_wrapper.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_ros_driver/dai_nodes/nn/mobilenet.hpp"
#include "depthai_ros_driver/dai_nodes/nn/segmentation.hpp"
#include "depthai_ros_driver/dai_nodes/nn/yolo.hpp"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"

namespace depthai_ros_driver {
namespace dai_nodes {
NNWrapper::NNWrapper(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
    ROS_DEBUG("Creating node %s base", daiNodeName.c_str());
    ph = std::make_unique<param_handlers::NNParamHandler>(daiNodeName);
    auto family = ph->getNNFamily(getROSNode());
    switch(family) {
        case param_handlers::nn::NNFamily::Yolo: {
            nnNode = std::make_unique<dai_nodes::nn::Yolo>(getName(), getROSNode(), pipeline);
            break;
        }
        case param_handlers::nn::NNFamily::Mobilenet: {
            nnNode = std::make_unique<dai_nodes::nn::Mobilenet>(getName(), getROSNode(), pipeline);
            break;
        }
        case param_handlers::nn::NNFamily::Segmentation: {
            nnNode = std::make_unique<dai_nodes::nn::Segmentation>(getName(), getROSNode(), pipeline);
            break;
        }
    }

    ROS_DEBUG("Base node %s created", daiNodeName.c_str());
}
void NNWrapper::setNames() {}

void NNWrapper::setXinXout(std::shared_ptr<dai::Pipeline> /*pipeline*/) {}

void NNWrapper::setupQueues(std::shared_ptr<dai::Device> device) {
    nnNode->setupQueues(device);
}
void NNWrapper::closeQueues() {
    nnNode->closeQueues();
}

void NNWrapper::link(const dai::Node::Input& in, int linkType) {
    nnNode->link(in, linkType);
}

dai::Node::Input NNWrapper::getInput(int linkType) {
    return nnNode->getInput(linkType);
}

void NNWrapper::updateParams(parametersConfig& config) {
    ph->setRuntimeParams(getROSNode(), config);
    nnNode->updateParams(config);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
