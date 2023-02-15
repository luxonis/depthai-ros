#include "depthai_ros_driver/dai_nodes/nn/spatial_yolo.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_ros_driver/dai_nodes/nn/nn_helpers.hpp"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn {

SpatialYolo::SpatialYolo(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> pipeline)
    : BaseNode(daiNodeName, node, pipeline) {
    ROS_DEBUG("Creating node %s", daiNodeName.c_str());
    setNames();
    yoloNode = pipeline->create<dai::node::YoloSpatialDetectionNetwork>();
    imageManip = pipeline->create<dai::node::ImageManip>();
    ph = std::make_unique<param_handlers::NNParamHandler>(daiNodeName);
    ph->declareParams(node, yoloNode, imageManip);
    imageManip->out.link(yoloNode->input);
    setXinXout(pipeline);
    ROS_DEBUG("Node %s created", daiNodeName.c_str());
}

void SpatialYolo::setNames() {
    nnQName = getName() + "_nn";
}

void SpatialYolo::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutNN = pipeline->create<dai::node::XLinkOut>();
    xoutNN->setStreamName(nnQName);
    yoloNode->out.link(xoutNN->input);
}

void SpatialYolo::setupQueues(std::shared_ptr<dai::Device> device) {
    nnQ = device->getOutputQueue(nnQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
    auto tfPrefix = std::string(getROSNode().getNamespace());
    tfPrefix.erase(0, 1);
    detConverter = std::make_unique<dai::ros::SpatialDetectionConverter>(
        tfPrefix + "_rgb_camera_optical_frame", imageManip->initialConfig.getResizeConfig().width, imageManip->initialConfig.getResizeConfig().height, false);
    nnQ->addCallback(std::bind(&SpatialYolo::yoloCB, this, std::placeholders::_1, std::placeholders::_2));
    detPub = getROSNode().advertise<vision_msgs::Detection3DArray>(getName() + "/detections", 10);
}
void SpatialYolo::closeQueues() {
    nnQ->close();
}

void SpatialYolo::yoloCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto inDet = std::dynamic_pointer_cast<dai::SpatialImgDetections>(data);
    std::deque<vision_msgs::Detection3DArray> deq;
    detConverter->toRosVisionMsg(inDet, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        detPub.publish(currMsg);
        deq.pop_front();
    }
}

void SpatialYolo::link(const dai::Node::Input& in, int /*linkType*/) {
    yoloNode->out.link(in);
}

dai::Node::Input SpatialYolo::getInput(int linkType) {
    if(linkType == static_cast<int>(nn_helpers::link_types::SpatialNNLinkType::input)) {
        return imageManip->inputImage;
    } else {
        return yoloNode->inputDepth;
    }
}

void SpatialYolo::updateParams(parametersConfig& config) {
    ph->setRuntimeParams(getROSNode(), config);
}
}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver