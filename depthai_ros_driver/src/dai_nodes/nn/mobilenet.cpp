#include "depthai_ros_driver/dai_nodes/nn/mobilenet.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn {

Mobilenet::Mobilenet(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
    ROS_DEBUG("Creating node %s", daiNodeName.c_str());
    setNames();
    mobileNode = pipeline->create<dai::node::MobileNetDetectionNetwork>();
    imageManip = pipeline->create<dai::node::ImageManip>();
    ph = std::make_unique<param_handlers::NNParamHandler>(daiNodeName);
    ph->declareParams(node, mobileNode, imageManip);
    imageManip->out.link(mobileNode->input);
    setXinXout(pipeline);
    ROS_DEBUG("Node %s created", daiNodeName.c_str());
}

void Mobilenet::setNames() {
    nnQName = getName() + "_nn";
}

void Mobilenet::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutNN = pipeline->create<dai::node::XLinkOut>();
    xoutNN->setStreamName(nnQName);
    mobileNode->out.link(xoutNN->input);
}

void Mobilenet::setupQueues(std::shared_ptr<dai::Device> device) {
    nnQ = device->getOutputQueue(nnQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
    auto tfPrefix = std::string(getROSNode().getNamespace());
    tfPrefix.erase(0, 1);
    detConverter = std::make_unique<dai::ros::ImgDetectionConverter>(
        tfPrefix + "_rgb_camera_optical_frame", imageManip->initialConfig.getResizeConfig().width, imageManip->initialConfig.getResizeConfig().height, false);
    nnQ->addCallback(std::bind(&Mobilenet::mobilenetCB, this, std::placeholders::_1, std::placeholders::_2));
    detPub = getROSNode().advertise<vision_msgs::Detection2DArray>(getName() + "/detections", 10);
}
void Mobilenet::closeQueues() {
    nnQ->close();
}

void Mobilenet::mobilenetCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto inDet = std::dynamic_pointer_cast<dai::ImgDetections>(data);
    std::deque<vision_msgs::Detection2DArray> deq;
    detConverter->toRosMsg(inDet, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        detPub.publish(currMsg);
        deq.pop_front();
    }
}

void Mobilenet::link(const dai::Node::Input& in, int /*linkType*/) {
    mobileNode->out.link(in);
}

dai::Node::Input Mobilenet::getInput(int /*linkType*/) {
    return imageManip->inputImage;
}

void Mobilenet::updateParams(parametersConfig& config) {
    ph->setRuntimeParams(getROSNode(), config);
}

}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver