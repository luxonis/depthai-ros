#include "depthai_ros_driver/dai_nodes/stereo.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
namespace depthai_ros_driver {
namespace dai_nodes {
Stereo::Stereo(const std::string& dai_node_name, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(dai_node_name, node, pipeline) {
    RCLCPP_INFO(node->get_logger(), "Creating node %s", dai_node_name.c_str());
    set_names();
    stereo_cam_node_ = pipeline->create<dai::node::StereoDepth>();
    param_handler_ = std::make_unique<param_handlers::StereoParamHandler>(dai_node_name);
    param_handler_->declareParams(node, stereo_cam_node_);
    set_xin_xout(pipeline);
    RCLCPP_INFO(node->get_logger(), "Node %s created", dai_node_name.c_str());
};
void Stereo::set_names() {
    stereo_q_name_ = getName() + "_stereo";
}

void Stereo::set_xin_xout(std::shared_ptr<dai::Pipeline> pipeline) {
    xout_stereo_ = pipeline->create<dai::node::XLinkOut>();
    xout_stereo_->setStreamName(stereo_q_name_);
    stereo_cam_node_->depth.link(xout_stereo_->input);
}

void Stereo::setupQueues(std::shared_ptr<dai::Device> device) {
    stereo_q_ = device->getOutputQueue(stereo_q_name_, param_handler_->get_param<int>(getROSNode(), "i_max_q_size"), false);
    stereo_q_->addCallback(std::bind(&Stereo::stereo_q_cb, this, std::placeholders::_1, std::placeholders::_2));
    stereo_pub_ = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");
}

void Stereo::stereo_q_cb(const std::string& name, const std::shared_ptr<dai::ADatatype>& data) {
    auto names = name;
    auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    cv::Mat cv_frame = frame->getCvFrame();
    auto curr_time = getROSNode()->get_clock()->now();
    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, cv_frame);
    img_bridge.toImageMsg(img_msg);
    stereo_pub_.publish(img_msg, stereo_info_);
}

void Stereo::link(const dai::Node::Input& in, int link_type) {
    stereo_cam_node_->depth.link(in);
}

dai::Node::Input Stereo::get_input(int link_type) {
    if(link_type == static_cast<int>(link_types::StereoLinkType::left)) {
        return stereo_cam_node_->left;
    } else {
        return stereo_cam_node_->right;
    }
}

void Stereo::updateParams(const std::vector<rclcpp::Parameter>& params) {
    auto ctrl = param_handler_->setRuntimeParams(params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
