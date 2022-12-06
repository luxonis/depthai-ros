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
    paramHandler = std::make_unique<param_handlers::MonoParamHandler>(daiNodeName);
    paramHandler->declareParams(node, monoCamNode, socket, sensor);
    setXinXout(pipeline);
    RCLCPP_INFO(node->get_logger(), "Node %s created", daiNodeName.c_str());
};
void Mono::setNames() {
    monoQName = getName() + "_mono";
    controlQName = getName() + "_control";
}

void Mono::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    if(paramHandler->get_param<bool>(getROSNode(), "i_publish_topic")) {
        xoutMono = pipeline->create<dai::node::XLinkOut>();
        xoutMono->setStreamName(monoQName);
        monoCamNode->out.link(xoutMono->input);
    }
    xinControl = pipeline->create<dai::node::XLinkIn>();
    xinControl->setStreamName(controlQName);
    xinControl->out.link(monoCamNode->inputControl);
}

void Mono::setupQueues(std::shared_ptr<dai::Device> device) {
    if(paramHandler->get_param<bool>(getROSNode(), "i_publish_topic")) {
        monoQ = device->getOutputQueue(monoQName, paramHandler->get_param<int>(getROSNode(), "i_max_q_size"), false);
        monoQ->addCallback(std::bind(&Mono::monoQCB, this, std::placeholders::_1, std::placeholders::_2));
        monoPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");
        auto calibHandler = device->readCalibration();
        monoInfo = dai::ros::calibrationToCameraInfo(calibHandler,
                                                     static_cast<dai::CameraBoardSocket>(paramHandler->get_param<int>(getROSNode(), "i_board_socket_id")),
                                                     paramHandler->get_param<int>(getROSNode(), "i_width"),
                                                     paramHandler->get_param<int>(getROSNode(), "i_height"));
    }
    controlQ = device->getInputQueue(controlQName);
}
void Mono::closeQueues() {
    if(paramHandler->get_param<bool>(getROSNode(), "i_publish_topic")) {
        monoQ->close();
    }
    controlQ->close();
}

void Mono::monoQCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data) {
    auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    cv::Mat cv_frame = frame->getCvFrame();
    auto curr_time = getROSNode()->get_clock()->now();
    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    header.stamp = curr_time;
    header.frame_id = std::string(getROSNode()->get_name()) + "_" + getName() + "_camera_optical_frame";
    monoInfo.header = header;
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
    auto ctrl = paramHandler->setRuntimeParams(getROSNode(), params);
    controlQ->send(ctrl);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
