#include "depthai_ros_driver/dai_nodes/nn/spatial_nn_wrapper.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai_ros_driver/dai_nodes/nn/spatial_detection.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "ros/node_handle.h"

namespace depthai_ros_driver {
namespace dai_nodes {
SpatialNNWrapper::SpatialNNWrapper(const std::string& daiNodeName,
                                   ros::NodeHandle node,
                                   std::shared_ptr<dai::Pipeline> pipeline,
                                   const dai::CameraBoardSocket& socket)
    : BaseNode(daiNodeName, node, pipeline) {
    ROS_DEBUG("Creating node %s base", daiNodeName.c_str());
    ph = std::make_unique<param_handlers::NNParamHandler>(node, daiNodeName, socket);
    auto family = ph->getNNFamily();
    switch(family) {
        case param_handlers::nn::NNFamily::Yolo: {
            nnNode = std::make_unique<dai_nodes::nn::SpatialDetection<dai::node::YoloSpatialDetectionNetwork>>(getName(), getROSNode(), pipeline);
            break;
        }
        case param_handlers::nn::NNFamily::Mobilenet: {
            nnNode = std::make_unique<dai_nodes::nn::SpatialDetection<dai::node::MobileNetSpatialDetectionNetwork>>(getName(), getROSNode(), pipeline);
            break;
        }
        case param_handlers::nn::NNFamily::Segmentation: {
            throw(std::runtime_error("Segmentation not supported for spatial network!"));
        }
    }

    ROS_DEBUG("Base node %s created", daiNodeName.c_str());
}

SpatialNNWrapper::~SpatialNNWrapper() = default;
void SpatialNNWrapper::setNames() {}

void SpatialNNWrapper::setXinXout(std::shared_ptr<dai::Pipeline> /*pipeline*/) {}

void SpatialNNWrapper::setupQueues(std::shared_ptr<dai::Device> device) {
    nnNode->setupQueues(device);
}
void SpatialNNWrapper::closeQueues() {
    nnNode->closeQueues();
}

void SpatialNNWrapper::link(dai::Node::Input in, int linkType) {
    nnNode->link(in, linkType);
}

dai::Node::Input SpatialNNWrapper::getInput(int linkType) {
    return nnNode->getInput(linkType);
}

void SpatialNNWrapper::updateParams(parametersConfig& config) {
    ph->setRuntimeParams(config);
    nnNode->updateParams(config);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
