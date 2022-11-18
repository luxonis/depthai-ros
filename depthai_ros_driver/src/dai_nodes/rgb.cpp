#include "depthai_ros_driver/dai_nodes/rgb.hpp"

#include "image_transport/camera_publisher.hpp"
namespace depthai_ros_driver {
namespace dai_nodes {
void RGB::initialize(const std::string& dai_node_name, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) {
    RCLCPP_INFO(node->get_logger(), "Creating node %s", dai_node_name);
    dai_node_name_ = dai_node_name;
    node_ = node;
    color_q_name_ = dai_node_name_ + "_color";
    preview_q_name_ = dai_node_name_ + "_preview";
    control_q_name_ = dai_node_name_ + "_control";
    params_handler_ = std::make_unique<param_handlers::RGBParamHandler>(dai_node_name);
    color_cam_node_ = pipeline->create<dai::node::ColorCamera>();
    params_handler_->declareParams(node_, color_cam_node_);
    xout_color_ = pipeline->create<dai::node::XLinkOut>();
    xout_color_->setStreamName(color_q_name_);
    xin_control_ = pipeline->create<dai::node::XLinkIn>();
    xin_control_->setStreamName(control_q_name_);
    color_cam_node_->video.link(xout_color_->input);
    xin_control_->out.link(color_cam_node_->inputControl);
    RCLCPP_INFO(node->get_logger(), "Node %s created", dai_node_name);
};

void RGB::setupQueues(std::shared_ptr<dai::Device> device) {
    color_q_ = device->getOutputQueue(color_q_name_, node_->get_parameter(dai_node_name_ + ".i_max_q_size").as_int(), false);
    color_q_->addCallback(std::bind(&RGB::color_q_cb, this, std::placeholders::_1, std::placeholders::_2));
    control_q_ = device->getInputQueue(control_q_name_);
    rgb_pub_ = image_transport::create_camera_publisher(node_, "~/" + dai_node_name_ + "/color/image_raw");
}

void RGB::color_q_cb(const std::string& name, const std::shared_ptr<dai::ADatatype>& data) {
    auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    cv::Mat cv_frame = frame->getCvFrame();
    auto curr_time = node_->get_clock()->now();
    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    img_bridge = cv_bridge::CvImage(header, encoding, cv_frame);
    img_bridge.toImageMsg(img_msg);
}

void RGB::updateParams(const std::vector<rclcpp::Parameter>& params) {
    auto ctrl = params_handler_->setRuntimeParams(params);
    control_q_->send(ctrl);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::dai_nodes::RGB, depthai_ros_driver::dai_nodes::BaseNode)
