#include "depthai_ros_driver/dai_nodes/mono.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
namespace depthai_ros_driver {
namespace daiNodes {
Mono::Mono(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_INFO(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    monoCamNode = pipeline->create<dai::node::MonoCamera>();
    paramHandler = std::make_unique<paramHandlers::MonoParamHandler>(daiNodeName);
    paramHandler->declareParams(node, monoCamNode);
    setXinXout(pipeline);
    RCLCPP_INFO(node->get_logger(), "Node %s created", daiNodeName.c_str());
    if(getName() == "mono_left") {
        monoCamNode->setBoardSocket(dai::CameraBoardSocket::LEFT);
    } else if(getName() == "mono_right") {
        monoCamNode->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    } else {
        monoCamNode->setBoardSocket(static_cast<dai::CameraBoardSocket>(paramHandler->get_param<int>(getROSNode(), "i_board_socket")));
    };
};
void Mono::setNames() {
    monoQName = getName() + "_mono";
    controlQName = getName() + "_control";
}

void Mono::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutMono = pipeline->create<dai::node::XLinkOut>();
    xoutMono->setStreamName(monoQName);
    xinControl = pipeline->create<dai::node::XLinkIn>();
    xinControl->setStreamName(controlQName);
    monoCamNode->out.link(xoutMono->input);
    xinControl->out.link(monoCamNode->inputControl);
}

void Mono::setupQueues(std::shared_ptr<dai::Device> device) {
    monoQ = device->getOutputQueue(monoQName, paramHandler->get_param<int>(getROSNode(), "i_max_q_size"), false);
    monoQ->addCallback(std::bind(&Mono::monoQCB, this, std::placeholders::_1, std::placeholders::_2));
    controlQ = device->getInputQueue(controlQName);
    monoPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");
}

void Mono::monoQCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data) {
    auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    cv::Mat cv_frame = frame->getCvFrame();
    auto curr_time = getROSNode()->get_clock()->now();
    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, cv_frame);
    img_bridge.toImageMsg(img_msg);
    monoPub.publish(img_msg, monoInfo);
}

void Mono::link(const dai::Node::Input& in, int /*linkType*/) {
    monoCamNode->out.link(in);
}

dai::Node::Input Mono::getInput(int linkType) {
    throw(std::runtime_error("Class Mono has no input."));
}

void Mono::updateParams(const std::vector<rclcpp::Parameter>& params) {
    auto ctrl = paramHandler->setRuntimeParams(getROSNode(),params);
    controlQ->send(ctrl);
}

}  // namespace daiNodes
}  // namespace depthai_ros_driver
