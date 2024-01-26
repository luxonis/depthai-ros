#pragma once

#include <memory>
#include <string>

#include "depthai/pipeline/Node.hpp"

namespace dai {
class Pipeline;
class Device;
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

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
    BaseNode(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline);
    virtual ~BaseNode();
    virtual void updateParams(const std::vector<rclcpp::Parameter>& params);
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
    void setROSNodePointer(rclcpp::Node* node);
    /**
     * @brief      Gets the ROS node pointer.
     *
     * @return     The ROS node pointer.
     */
    rclcpp::Node* getROSNode();
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
    bool ipcEnabled();

   private:
    rclcpp::Node* baseNode;
    std::string baseDAINodeName;
    bool intraProcessEnabled;
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver