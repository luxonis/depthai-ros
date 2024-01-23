#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/parametersConfig.h"

namespace dai {
class Pipeline;
class Device;
}  // namespace dai

namespace ros {
class NodeHandle;
}  // namespace ros

namespace depthai_ros_driver {
namespace param_handlers {
class NNParamHandler;
}

namespace dai_nodes {

class NNWrapper : public BaseNode {
   public:
    explicit NNWrapper(const std::string& daiNodeName,
                       ros::NodeHandle node,
                       std::shared_ptr<dai::Pipeline> pipeline,
                       const dai::CameraBoardSocket& socket = dai::CameraBoardSocket::CAM_A);
    ~NNWrapper();
    void updateParams(parametersConfig& config) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0);
    virtual void setNames();
    virtual void setXinXout(std::shared_ptr<dai::Pipeline> pipeline);
    void closeQueues() override;

   private:
    std::unique_ptr<param_handlers::NNParamHandler> ph;
    std::unique_ptr<BaseNode> nnNode;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver