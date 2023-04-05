#pragma once

#include <memory>
#include <string>

#include "depthai/pipeline/Node.hpp"
#include "depthai_ros_driver/parametersConfig.h"

namespace dai {
class Pipeline;
class Device;
}  // namespace dai

namespace ros {
class NodeHandle;
class Parameter;
}  // namespace rclcpp


namespace depthai_ros_driver {
namespace dai_nodes {
class BaseNode {
   public:
    BaseNode(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> /*pipeline*/) {
        setNodeName(daiNodeName);
        setROSNodePointer(node);
    };
    virtual ~BaseNode(){};
    virtual void updateParams(parametersConfig& config) = 0;
    virtual void link(dai::Node::Input in, int linkType = 0) = 0;
    virtual dai::Node::Input getInput(int /*linkType = 0*/) {
        throw(std::runtime_error("getInput() not implemented"));
    };
    virtual void setupQueues(std::shared_ptr<dai::Device> device) = 0;
    virtual void setNames() = 0;
    virtual void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) = 0;
    virtual void closeQueues() = 0;

    void setNodeName(const std::string& daiNodeName);
    void setROSNodePointer(ros::NodeHandle node);
    ros::NodeHandle getROSNode();
    std::string getName();
    std::string getTFPrefix(const std::string& frameName = "");

   private:
    ros::NodeHandle baseNode;
    std::string baseDAINodeName;
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver