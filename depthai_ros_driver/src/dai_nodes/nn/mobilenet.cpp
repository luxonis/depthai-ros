#include "depthai_ros_driver/dai_nodes/nn/mobilenet.hpp"

#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn {

Mobilenet::Mobilenet(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    mobileNode = pipeline->create<dai::node::MobileNetDetectionNetwork>();
    imageManip = pipeline->create<dai::node::ImageManip>();
    ph = std::make_unique<param_handlers::NNParamHandler>(node, daiNodeName);
    ph->declareParams(mobileNode, imageManip);
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
    imageManip->out.link(mobileNode->input);
    setXinXout(pipeline);
}
Mobilenet::~Mobilenet() = default;
void Mobilenet::setNames() {
    nnQName = getName() + "_nn";
}

void Mobilenet::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutNN = pipeline->create<dai::node::XLinkOut>();
    xoutNN->setStreamName(nnQName);
    xoutPrevNN = pipeline->create<dai::node::XLinkOut>();
    xoutPrevNN->setStreamName("prev_q_nn");
    mobileNode->out.link(xoutNN->input);
    mobileNode->passthrough.link(xoutPrevNN->input);
}

void Mobilenet::setupQueues(std::shared_ptr<dai::Device> device) {
    nnQ = device->getOutputQueue(nnQName, ph->getParam<int>("i_max_q_size"), false);
    prevNNQ = device->getOutputQueue("prev_q_nn", 4, false);
    auto tfPrefix = std::string(getROSNode()->get_name());
    detConverter = std::make_unique<dai::ros::ImgDetectionConverter>(
        tfPrefix + "_rgb_camera_optical_frame", imageManip->initialConfig.getResizeConfig().width, imageManip->initialConfig.getResizeConfig().height, false);
    nnQ->addCallback(std::bind(&Mobilenet::mobilenetCB, this, std::placeholders::_1, std::placeholders::_2));
    detPub = getROSNode()->create_publisher<vision_msgs::msg::Detection2DArray>("~/" + getName() + "/detections", 10);
    imageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
    nnPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");
    infoManager = std::make_shared<camera_info_manager::CameraInfoManager>(
        getROSNode()->create_sub_node(std::string(getROSNode()->get_name()) + "/" + getName()).get(), "/" + getName());
    infoManager->setCameraInfo(sensor_helpers::getCalibInfo(getROSNode()->get_logger(),
                                                                    *imageConverter,
                                                                    device,
                                                                    dai::CameraBoardSocket::RGB,
                                                                    640,
                                                                    640));
    prevNNQ->addCallback(std::bind(sensor_helpers::imgCB,
                                   std::placeholders::_1,
                                   std::placeholders::_2,
                                   *imageConverter,
                                   nnPub,
                                   infoManager));
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
            int class_id = stoi(currMsg.detections[0].results[0].hypothesis.class_id);
            currMsg.detections[0].results[0].hypothesis.class_id = ph->getParam<std::vector<std::string>>("i_label_map")[class_id];
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
    ph->setRuntimeParams(params);
}

}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver