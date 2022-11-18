#pragma once

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/rgb_param_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "image_transport/camera_publisher.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
class RGB : protected BaseNode {
   public:
    void initialize(const std::string& dai_node_name, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) override;
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input & in) override;
    void link_preview(dai::Node::Input & in);
   private:
    void color_q_cb(const std::string &name, const std::shared_ptr<dai::ADatatype> &data);
    image_transport::CameraPublisher rgb_pub_, preview_pub_;
    sensor_msgs::msg::CameraInfo rgb_info_;
    std::shared_ptr<dai::node::ColorCamera> color_cam_node_;
    std::unique_ptr<param_handlers::RGBParamHandler> param_handler;
    std::shared_ptr<dai::DataOutputQueue> color_q_;
    std::shared_ptr<dai::DataInputQueue> control_q_;
    std::shared_ptr<dai::node::XLinkOut> xout_color_;
    std::shared_ptr<dai::node::XLinkIn> xin_control_;
    std::string color_q_name_, preview_q_name_, control_q_name_;
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver