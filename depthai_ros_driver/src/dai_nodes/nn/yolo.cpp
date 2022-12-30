#include "depthai_ros_driver/dai_nodes/nn/yolo.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn {

Yolo::Yolo(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
    ROS_DEBUG("Creating node %s", daiNodeName.c_str());
    setNames();
    yoloNode = pipeline->create<dai::node::YoloDetectionNetwork>();
    imageManip = pipeline->create<dai::node::ImageManip>();
    ph = std::make_unique<param_handlers::NNParamHandler>(daiNodeName);
    ph->declareParams(node, yoloNode, imageManip);
    imageManip->out.link(yoloNode->input);
    setXinXout(pipeline);
    ROS_DEBUG("Node %s created", daiNodeName.c_str());
}

void Yolo::setNames() {
    nnQName = getName() + "_nn";
}

void Yolo::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutNN = pipeline->create<dai::node::XLinkOut>();
    xoutNN->setStreamName(nnQName);
    yoloNode->out.link(xoutNN->input);
}

void Yolo::setupQueues(std::shared_ptr<dai::Device> device) {
    nnQ = device->getOutputQueue(nnQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
    auto tfPrefix = std::string(getROSNode().getNamespace());
    tfPrefix.erase(0, 1);
    detConverter = std::make_unique<dai::ros::ImgDetectionConverter>(
        tfPrefix + "_rgb_camera_optical_frame", imageManip->initialConfig.getResizeConfig().width, imageManip->initialConfig.getResizeConfig().height, false);
    nnQ->addCallback(std::bind(&Yolo::yoloCB, this, std::placeholders::_1, std::placeholders::_2));
    detPub = getROSNode().advertise<vision_msgs::Detection2DArray>(getName() + "/detections", 10);
}
void Yolo::closeQueues() {
    nnQ->close();
}

void Yolo::yoloCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto inDet = std::dynamic_pointer_cast<dai::ImgDetections>(data);
    std::deque<vision_msgs::Detection2DArray> deq;
    detConverter->toRosMsg(inDet, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();

        detPub.publish(currMsg);
        deq.pop_front();
    }
}

void Yolo::link(const dai::Node::Input& in, int /*linkType*/) {
    yoloNode->out.link(in);
}

dai::Node::Input Yolo::getInput(int /*linkType*/) {
    return imageManip->inputImage;
}

void Yolo::updateParams(parametersConfig& config) {
    ph->setRuntimeParams(getROSNode(), config);
}
}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver