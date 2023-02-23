#include "depthai_ros_driver/dai_nodes/nn/spatial_mobilenet.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_ros_driver/dai_nodes/nn/nn_helpers.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn {

SpatialMobilenet::SpatialMobilenet(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    mobileNode = pipeline->create<dai::node::MobileNetSpatialDetectionNetwork>();
    imageManip = pipeline->create<dai::node::ImageManip>();
    ph = std::make_unique<param_handlers::NNParamHandler>(daiNodeName);
    ph->declareParams(node, mobileNode, imageManip);
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
    imageManip->out.link(mobileNode->input);
    setXinXout(pipeline);
}

void SpatialMobilenet::setNames() {
    nnQName = getName() + "_nn";
}

void SpatialMobilenet::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutNN = pipeline->create<dai::node::XLinkOut>();
    xoutNN->setStreamName(nnQName);
    mobileNode->out.link(xoutNN->input);
}

void SpatialMobilenet::setupQueues(std::shared_ptr<dai::Device> device) {
    nnQ = device->getOutputQueue(nnQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
    auto tfPrefix = std::string(getROSNode()->get_name());
    detConverter = std::make_unique<dai::ros::SpatialDetectionConverter>(
        tfPrefix + "_rgb_camera_optical_frame", imageManip->initialConfig.getResizeConfig().width, imageManip->initialConfig.getResizeConfig().height, false);
    nnQ->addCallback(std::bind(&SpatialMobilenet::mobilenetCB, this, std::placeholders::_1, std::placeholders::_2));
    detPub = getROSNode()->create_publisher<vision_msgs::msg::Detection3DArray>("~/" + getName() + "/detections", 10);
}
void SpatialMobilenet::closeQueues() {
    nnQ->close();
}

void SpatialMobilenet::mobilenetCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto inDet = std::dynamic_pointer_cast<dai::SpatialImgDetections>(data);
    std::deque<vision_msgs::msg::Detection3DArray> deq;
    detConverter->toRosVisionMsg(inDet, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        detPub->publish(currMsg);
        deq.pop_front();
    }
}

void SpatialMobilenet::link(const dai::Node::Input& in, int /*linkType*/) {
    dai::Node::Input& input = const_cast<dai::Node::Input&>(in);
    mobileNode->out.link(input);
}

dai::Node::Input SpatialMobilenet::getInput(int linkType) {
    if(linkType == static_cast<int>(nn_helpers::link_types::SpatialNNLinkType::input)) {
        return imageManip->inputImage;
    } else {
        return mobileNode->inputDepth;
    }
}

void SpatialMobilenet::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(getROSNode(), params);
}

}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver