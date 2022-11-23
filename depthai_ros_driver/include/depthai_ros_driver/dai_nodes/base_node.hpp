#pragma once

#include "depthai/depthai.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "rclcpp/rclcpp.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
class BaseNode {
   public:
    BaseNode(){};
    virtual ~BaseNode(){};
    virtual void initialize(const std::string& dai_node_name, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) = 0;
    virtual void updateParams(const std::vector<rclcpp::Parameter>& params) = 0;
    virtual void link(dai::Node::Input& in, int link_type=0) = 0;
    virtual void setupQueues(std::shared_ptr<dai::Device> device) = 0;
    virtual void set_names(const std::string& dai_node_name) = 0;
    virtual void set_xin_xout(std::shared_ptr<dai::Pipeline> pipeline) = 0;

    void setNodeName(const std::string& dai_node_name){dai_node_name_ = dai_node_name;};
    void setROSNodePointer(rclcpp::Node* node){node_ = node;};
    rclcpp::Node* getROSNode(){return node_;};
    std::string getName(){return dai_node_name_;};

   private:
    rclcpp::Node* node_;
    std::string dai_node_name_;
};
class BaseNodeFactory {
   public:
    virtual std::unique_ptr<BaseNode> create() = 0;
    virtual ~BaseNodeFactory(){};
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver