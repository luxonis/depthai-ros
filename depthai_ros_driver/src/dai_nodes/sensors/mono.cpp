#include "depthai_ros_driver/dai_nodes/sensors/mono.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_bridge/ImageConverter.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
Mono::Mono(const std::string& daiNodeName,
           rclcpp::Node* node,
           std::shared_ptr<dai::Pipeline> pipeline,
           dai::CameraBoardSocket socket,
           dai_nodes::sensor_helpers::ImageSensor sensor)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_INFO(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    monoCamNode = pipeline->create<dai::node::MonoCamera>();
    ph = std::make_unique<param_handlers::MonoParamHandler>(daiNodeName);
    ph->declareParams(node, monoCamNode, socket, sensor);
    setXinXout(pipeline);
    RCLCPP_INFO(node->get_logger(), "Node %s created", daiNodeName.c_str());
};
void Mono::setNames() {
    monoQName = getName() + "_mono";
    controlQName = getName() + "_control";
}

void Mono::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    if(ph->getParam<bool>(getROSNode(), "i_publish_topic")) {
        xoutMono = pipeline->create<dai::node::XLinkOut>();
        xoutMono->setStreamName(monoQName);
        monoCamNode->out.link(xoutMono->input);
    }
    xinControl = pipeline->create<dai::node::XLinkIn>();
    xinControl->setStreamName(controlQName);
    xinControl->out.link(monoCamNode->inputControl);
}

void Mono::setupQueues(std::shared_ptr<dai::Device> device) {
    if(ph->getParam<bool>(getROSNode(), "i_publish_topic")) {
        monoQ = device->getOutputQueue(monoQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
        monoQ->addCallback(std::bind(&Mono::monoQCB, this, std::placeholders::_1, std::placeholders::_2));
        auto tfPrefix = std::string(getROSNode()->get_name()) + "_" + getName();
        imageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
        monoPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");
        auto calibHandler = device->readCalibration();
        monoInfo = dai::ros::calibrationToCameraInfo(calibHandler,
                                                     static_cast<dai::CameraBoardSocket>(ph->getParam<int>(getROSNode(), "i_board_socket_id")),
                                                     ph->getParam<int>(getROSNode(), "i_width"),
                                                     ph->getParam<int>(getROSNode(), "i_height"));
    }
    controlQ = device->getInputQueue(controlQName);
}
void Mono::closeQueues() {
    if(ph->getParam<bool>(getROSNode(), "i_publish_topic")) {
        monoQ->close();
    }
    controlQ->close();
}

void Mono::monoQCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto img = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    std::deque<sensor_msgs::msg::Image> deq;
    imageConverter->toRosMsg(img, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        monoInfo.header =currMsg.header;
        monoPub.publish(currMsg, monoInfo);
        deq.pop_front();
    }
}

void Mono::link(const dai::Node::Input& in, int /*linkType*/) {
    monoCamNode->out.link(in);
}

dai::Node::Input Mono::getInput(int linkType) {
    throw(std::runtime_error("Class Mono has no input."));
}

void Mono::updateParams(const std::vector<rclcpp::Parameter>& params) {
    auto ctrl = ph->setRuntimeParams(getROSNode(), params);
    controlQ->send(ctrl);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver