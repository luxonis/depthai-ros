#include "cv_bridge/cv_bridge.h"

#include "depthai_ros_driver/dai_nodes/rgb.hpp"

#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
namespace depthai_ros_driver {
namespace dai_nodes {
void RGB::initialize(const std::string& dai_node_name, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) {
    RCLCPP_INFO(node->get_logger(), "Creating node %s", dai_node_name.c_str());
    node_ = node;
    set_names(dai_node_name);
    color_cam_node_ = pipeline->create<dai::node::ColorCamera>();
    param_handler_ = std::make_unique<param_handlers::RGBParamHandler>(dai_node_name);
    param_handler_->declareParams(node_, color_cam_node_);
    set_xin_xout(pipeline);
    RCLCPP_INFO(node->get_logger(), "Node %s created", dai_node_name.c_str());
};
void RGB::set_names(const std::string & dai_node_name){
    dai_node_name_ = dai_node_name;
    color_q_name_ = dai_node_name_ + "_color";
    preview_q_name_ = dai_node_name_ + "_preview";
    control_q_name_ = dai_node_name_ + "_control";
}

void RGB::set_xin_xout(std::shared_ptr<dai::Pipeline> pipeline){
    xout_color_ = pipeline->create<dai::node::XLinkOut>();
    xout_color_->setStreamName(color_q_name_);
    xin_control_ = pipeline->create<dai::node::XLinkIn>();
    xin_control_->setStreamName(control_q_name_);
    color_cam_node_->video.link(xout_color_->input);
    xin_control_->out.link(color_cam_node_->inputControl);
    if (param_handler_->get_param<bool>(node_, "i_enable_preview")){
        xout_preview_ = pipeline->create<dai::node::XLinkOut>();
        color_cam_node_->preview.link(xout_preview_->input);
    }
}

void RGB::setupQueues(std::shared_ptr<dai::Device> device) {
    color_q_ = device->getOutputQueue(color_q_name_, param_handler_->get_param<int>(node_,"i_max_q_size"), false);
    color_q_->addCallback(std::bind(&RGB::color_q_cb, this, std::placeholders::_1, std::placeholders::_2));
    control_q_ = device->getInputQueue(control_q_name_);
    rgb_pub_ = image_transport::create_camera_publisher(node_, "~/" + dai_node_name_ + "/color/image_raw");

    if (param_handler_->get_param<bool>(node_, "i_enable_preview")){
        preview_q_ = device->getOutputQueue(preview_q_name_, param_handler_->get_param<int>(node_,"i_max_q_size"), false);
        preview_q_->addCallback(std::bind(&RGB::color_q_cb, this, std::placeholders::_1, std::placeholders::_2));
        preview_pub_ = image_transport::create_camera_publisher(node_, "~/" + dai_node_name_ + "/preview/image_raw");
    }
}

void RGB::color_q_cb(const std::string & name, const std::shared_ptr<dai::ADatatype> & data) {
    auto names = name;
    auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    cv::Mat cv_frame = frame->getCvFrame();
    auto curr_time = node_->get_clock()->now();
    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cv_frame);
    img_bridge.toImageMsg(img_msg);
    rgb_pub_.publish(img_msg, rgb_info_);
}


void RGB::link(dai::Node::Input &in){
    color_cam_node_->video.link(in);
}

void RGB::link_preview(dai::Node::Input &in){
    color_cam_node_->preview.link(in);
}

void RGB::set_socket(dai::CameraBoardSocket socket){
    color_cam_node_->setBoardSocket(socket);
}

void RGB::updateParams(const std::vector<rclcpp::Parameter>& params) {
    auto ctrl = param_handler_->setRuntimeParams(params);
    control_q_->send(ctrl);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
