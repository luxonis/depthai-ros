#pragma once

#include "depthai/depthai.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "rclcpp/rclcpp.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
class BaseNode {
   public:
    BaseNode(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> /*pipeline*/) {
        setNodeName(daiNodeName);
        setROSNodePointer(node);
    };
    virtual ~BaseNode(){};
    virtual void updateParams(const std::vector<rclcpp::Parameter>& params) = 0;
    virtual void link(const dai::Node::Input& in, int linkType = 0) = 0;
    virtual dai::Node::Input getInput(int /*linkType = 0*/) {
        throw(std::runtime_error("getInput() not implemented"));
    };
    virtual void setupQueues(std::shared_ptr<dai::Device> device) = 0;
    virtual void setNames() = 0;
    virtual void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) = 0;
    virtual void closeQueues() = 0;

    void setNodeName(const std::string& daiNodeName) {
        baseDAINodeName = daiNodeName;
    };
    void setROSNodePointer(rclcpp::Node* node) {
        baseNode = node;
    };
    rclcpp::Node* getROSNode() {
        return baseNode;
    };
    std::string getName() {
        return baseDAINodeName;
    };
    std::string getTFPrefix(const std::string& frameName = "") {
        return std::string(getROSNode()->get_name()) + "_" + frameName;
    }

   private:
    rclcpp::Node* baseNode;
    std::string baseDAINodeName;
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver