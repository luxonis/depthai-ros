#pragma once

#include "depthai/depthai.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/Node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
class BaseNode {
   public:
    BaseNode(){};
    virtual ~BaseNode() {};
    virtual void initialize(const std::string &dai_node_name, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline)=0;
    virtual void updateParams(const std::vector<rclcpp::Parameter>& params)=0;
    virtual void link(dai::Node::Input & in)=0;
    virtual void setupQueues(std::shared_ptr<dai::Device> device)=0;
   protected:
    rclcpp::Node* node_;
    std::string dai_node_name_;
    virtual void set_names(const std::string & dai_node_name)=0;
    virtual void set_xin_xout(std::shared_ptr<dai::Pipeline> pipeline)=0;
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver