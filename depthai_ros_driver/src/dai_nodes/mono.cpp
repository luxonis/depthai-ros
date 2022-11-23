#include "depthai_ros_driver/dai_nodes/mono.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
namespace depthai_ros_driver {
namespace dai_nodes {
void Mono::initialize(const std::string& dai_node_name, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) {
    RCLCPP_INFO(node->get_logger(), "Creating node %s", dai_node_name.c_str());
    setROSNodePointer(node);
    set_names(dai_node_name);
    mono_cam_node_ = pipeline->create<dai::node::MonoCamera>();
    param_handler_ = std::make_unique<param_handlers::MonoParamHandler>(dai_node_name);
    param_handler_->declareParams(node, mono_cam_node_);
    set_xin_xout(pipeline);
    RCLCPP_INFO(node->get_logger(), "Node %s created", dai_node_name.c_str());
    if (getName()=="mono_left"){
    mono_cam_node_->setBoardSocket(dai::CameraBoardSocket::LEFT);
    }
    else{
        mono_cam_node_->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    }
};
void Mono::set_names(const std::string& dai_node_name) {
    setNodeName(dai_node_name);
    mono_q_name_ = getName() + "_mono";
    control_q_name_ = getName() + "_control";
}

void Mono::set_xin_xout(std::shared_ptr<dai::Pipeline> pipeline) {
    xout_mono_ = pipeline->create<dai::node::XLinkOut>();
    xout_mono_->setStreamName(mono_q_name_);
    xin_control_ = pipeline->create<dai::node::XLinkIn>();
    xin_control_->setStreamName(control_q_name_);
    mono_cam_node_->out.link(xout_mono_->input);
    xin_control_->out.link(mono_cam_node_->inputControl);
}

void Mono::setupQueues(std::shared_ptr<dai::Device> device) {
    mono_q_ = device->getOutputQueue(mono_q_name_, param_handler_->get_param<int>(getROSNode(), "i_max_q_size"), false);
    mono_q_->addCallback(std::bind(&Mono::mono_q_cb, this, std::placeholders::_1, std::placeholders::_2));
    control_q_ = device->getInputQueue(control_q_name_);
    mono_pub_ = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");
}

void Mono::mono_q_cb(const std::string& name, const std::shared_ptr<dai::ADatatype>& data) {
    auto names = name;
    auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    cv::Mat cv_frame = frame->getCvFrame();
    auto curr_time = getROSNode()->get_clock()->now();
    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, cv_frame);
    img_bridge.toImageMsg(img_msg);
    mono_pub_.publish(img_msg, mono_info_);
}

void Mono::link(dai::Node::Input& in, int link_type) {
    mono_cam_node_->out.link(in);
}

void Mono::updateParams(const std::vector<rclcpp::Parameter>& params) {
    auto ctrl = param_handler_->setRuntimeParams(params);
    control_q_->send(ctrl);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
