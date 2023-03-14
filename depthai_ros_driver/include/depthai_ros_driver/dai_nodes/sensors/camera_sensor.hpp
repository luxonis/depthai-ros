#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"

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

class CameraSensor : public BaseNode {
   public:
    explicit CameraSensor(const std::string& daiNodeName,
                          rclcpp::Node* node,
                          std::shared_ptr<dai::Pipeline> pipeline,
                          std::shared_ptr<dai::Device> device,
                          dai::CameraBoardSocket socket,
                          bool publish = true);
    ~CameraSensor();
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    std::unique_ptr<BaseNode> sensorNode;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver