#include "depthai_ros_driver/dai_nodes/nn/mobilenet.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn {

Mobilenet::Mobilenet(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    mobileNode = pipeline->create<dai::node::MobileNetDetectionNetwork>();
    imageManip = pipeline->create<dai::node::ImageManip>();
    ph = std::make_unique<param_handlers::NNParamHandler>(daiNodeName);
    ph->declareParams(node, mobileNode, imageManip);
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
    imageManip->out.link(mobileNode->input);
    setXinXout(pipeline);
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
    auto tfPrefix = std::string(getROSNode()->get_name());
    detConverter = std::make_unique<dai::ros::ImgDetectionConverter>(
        tfPrefix + "_rgb_camera_optical_frame", imageManip->initialConfig.getResizeConfig().width, imageManip->initialConfig.getResizeConfig().height, false);
    nnQ->addCallback(std::bind(&Mobilenet::mobilenetCB, this, std::placeholders::_1, std::placeholders::_2));
    detPub = getROSNode()->create_publisher<vision_msgs::msg::Detection2DArray>("~/" + getName() + "/detections", 10);
}
void Mobilenet::closeQueues() {
    nnQ->close();
}

void Mobilenet::mobilenetCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto inDet = std::dynamic_pointer_cast<dai::ImgDetections>(data);
    std::deque<vision_msgs::msg::Detection2DArray> deq;
    detConverter->toRosMsg(inDet, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        if(currMsg.detections.size() > 0) {
            int class_id = stoi(currMsg.detections[0].results[0].id);
            currMsg.detections[0].results[0].id = ph->getParam<std::vector<std::string>>(getROSNode(), "i_label_map")[class_id];
        }
        detPub->publish(currMsg);
        deq.pop_front();
    }
}

void Mobilenet::link(const dai::Node::Input& in, int /*linkType*/) {
    mobileNode->out.link(in);
}

dai::Node::Input Mobilenet::getInput(int /*linkType*/) {
    return imageManip->inputImage;
}

void Mobilenet::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(getROSNode(), params);
}

}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver