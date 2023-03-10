#include "depthai_ros_driver/dai_nodes/nn/yolo.hpp"

#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn {

Yolo::Yolo(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_INFO(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    yoloNode = pipeline->create<dai::node::YoloDetectionNetwork>();
    imageManip = pipeline->create<dai::node::ImageManip>();
    ph = std::make_unique<param_handlers::NNParamHandler>(daiNodeName);
    ph->declareParams(node, yoloNode, imageManip);
    RCLCPP_INFO(node->get_logger(), "Node %s created", daiNodeName.c_str());
    imageManip->out.link(yoloNode->input);
    setXinXout(pipeline);
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
    auto tfPrefix = std::string(getROSNode()->get_name());
    detConverter = std::make_unique<dai::ros::ImgDetectionConverter>(
        tfPrefix + "_rgb_camera_optical_frame", imageManip->initialConfig.getResizeConfig().width, imageManip->initialConfig.getResizeConfig().height, false);
    nnQ->addCallback(std::bind(&Yolo::yoloCB, this, std::placeholders::_1, std::placeholders::_2));
    detPub = getROSNode()->create_publisher<vision_msgs::msg::Detection2DArray>("~/" + getName() + "/detections", 10);
}
void Yolo::closeQueues() {
    nnQ->close();
}

void Yolo::yoloCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto inDet = std::dynamic_pointer_cast<dai::ImgDetections>(data);
    std::deque<vision_msgs::msg::Detection2DArray> deq;
    detConverter->toRosMsg(inDet, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        if(currMsg.detections.size() > 0) {
            int class_id = stoi(currMsg.detections[0].results[0].hypothesis.class_id);
            currMsg.detections[0].results[0].hypothesis.class_id = ph->getParam<std::vector<std::string>>(getROSNode(), "i_label_map")[class_id];
        }
        detPub->publish(currMsg);
        deq.pop_front();
    }
}

void Yolo::link(const dai::Node::Input& in, int /*linkType*/) {
    yoloNode->out.link(in);
}

dai::Node::Input Yolo::getInput(int /*linkType*/) {
    return imageManip->inputImage;
}

void Yolo::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(getROSNode(), params);
}
}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver