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
}  // namespace ros

namespace depthai_ros_driver {
namespace dai_nodes {
class BaseNode {
   public:
    /**
     * @brief      Constructor of the class BaseNode. Creates a node in the pipeline.
     *
     * @param[in]  daiNodeName  The dai node name
     * @param      node         The node
     * @param      pipeline     The pipeline
     */
    BaseNode(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> /*pipeline*/);
    virtual ~BaseNode();
    virtual void updateParams(parametersConfig& config);
    virtual void link(dai::Node::Input in, int linkType = 0);
    virtual dai::Node::Input getInput(int linkType = 0);
    virtual void setupQueues(std::shared_ptr<dai::Device> device) = 0;
    /**
     * @brief      Sets the names of the queues.
     */
    virtual void setNames() = 0;
    /**
     * @brief      Link inputs and outputs.
     *
     * @param      pipeline  The pipeline
     */
    virtual void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) = 0;
    virtual void closeQueues() = 0;

    void setNodeName(const std::string& daiNodeName);
    void setROSNodePointer(ros::NodeHandle node);
    /**
     * @brief      Gets the ROS node pointer.
     *
     * @return     The ROS node pointer.
     */
    ros::NodeHandle getROSNode();
    /**
     * @brief      Gets the name of the node.
     *
     * @return     The name.
     */
    std::string getName();
    /**
     * @brief    Append ROS node name to the frameName given.
     *
     * @param[in]  frameName  The frame name
     */
    std::string getTFPrefix(const std::string& frameName = "");

   private:
    ros::NodeHandle baseNode;
    std::string baseDAINodeName;
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver