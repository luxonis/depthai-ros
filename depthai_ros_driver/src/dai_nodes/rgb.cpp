#include "depthai_ros_driver/dai_nodes/rgb.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
void RGB::initialize(const std::string &name, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) {
    params_handler_ = std::make_unique<param_handlers::RGBParamHandler>();
    color_cam_node_ = pipeline->create<dai::node::ColorCamera>();
    params_handler_->declareParams(name, node, color_cam_node_);
    xlink_out_ = pipeline->create<dai::node::XLinkOut>();
    xlink_out_->
    readout_thread_ = std::thread();
};
void RGB::updateParams(const std::vector<rclcpp::Parameter>& params){
    auto control = params_handler_->setRuntimeParams(params);
    
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::dai_nodes::RGB, depthai_ros_driver::dai_nodes::BaseNode)
