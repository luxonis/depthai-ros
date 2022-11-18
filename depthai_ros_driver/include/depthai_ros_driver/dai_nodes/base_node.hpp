#pragma once

#include "depthai/depthai.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/Node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "depthai_ros_driver/camera.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
class BaseNode {
   public:
    virtual void initialize(const std::string &dai_node_name, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline);
    virtual void updateParams(const std::vector<rclcpp::Parameter>& params);
    virtual void link(dai::Node::Input & in);
    virtual void setupQueues(std::shared_ptr<dai::Device> device);
   protected:
    rclcpp::Node* node_;
    std::unique_ptr<param_handlers::BaseParamHandler> params_handler_;
    std::string dai_node_name_;
    BaseNode(){};
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver