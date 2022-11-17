#pragma once

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/rgb_param_handler.hpp"
#include "rclcpp/rclcpp.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
class RGB : protected BaseNode {
   public:
    void initialize(rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) override;
    void declareParams(rclcpp::Node* node) override;
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    protected:
        std::shared_ptr<dai::node::ColorCamera> color_cam_node_;
        std::unique_ptr<param_handlers::RGBParamHandler> param_handler;
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver