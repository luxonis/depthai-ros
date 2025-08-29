#include "depthai_ros_driver/dai_nodes/nn/spatial_nn_wrapper.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai_ros_driver/dai_nodes/nn/spatial_detection.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
SpatialNNWrapper::SpatialNNWrapper(const std::string& daiNodeName,
                                   std::shared_ptr<rclcpp::Node> node,
                                   std::shared_ptr<dai::Pipeline> pipeline,
                                   const std::string& deviceName,
                                   bool rsCompat,
                                   SensorWrapper& camNode,
                                   Stereo& stereoNode,
                                   const dai::CameraBoardSocket& socket)
    : BaseNode(daiNodeName, node, pipeline, deviceName, rsCompat) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s base", daiNodeName.c_str());
    ph = std::make_unique<param_handlers::NNParamHandler>(node, daiNodeName, deviceName, rsCompat, socket);
    auto family = ph->getNNFamily();
    switch(family) {
        case param_handlers::nn::NNFamily::Detection: {
            nnNode = std::make_unique<dai_nodes::nn::SpatialDetection>(getName(), getROSNode(), pipeline, deviceName, rsCompat, camNode, stereoNode, socket);
            break;
        }
        case param_handlers::nn::NNFamily::Segmentation: {
            throw(std::runtime_error("Segmentation not supported for spatial network!"));
        }
    }

    RCLCPP_DEBUG(node->get_logger(), "Base node %s created", daiNodeName.c_str());
}
SpatialNNWrapper::~SpatialNNWrapper() = default;
void SpatialNNWrapper::setNames() {}

void SpatialNNWrapper::setInOut(std::shared_ptr<dai::Pipeline> /*pipeline*/) {}

void SpatialNNWrapper::setupQueues(std::shared_ptr<dai::Device> device) {
    nnNode->setupQueues(device);
}
void SpatialNNWrapper::closeQueues() {
    nnNode->closeQueues();
}

void SpatialNNWrapper::link(dai::Node::Input& in, int linkType) {
    nnNode->link(in, linkType);
}

dai::Node::Input& SpatialNNWrapper::getInput(int linkType) {
    return nnNode->getInput(linkType);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
