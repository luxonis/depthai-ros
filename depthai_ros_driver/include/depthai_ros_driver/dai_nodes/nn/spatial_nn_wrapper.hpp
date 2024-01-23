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
class DataOutputQueue;
class ADatatype;
}  // namespace dai

namespace ros {
class NodeHandle;
}  // namespace ros

namespace depthai_ros_driver {
namespace param_handlers {
class NNParamHandler;
}
namespace dai_nodes {

class SpatialNNWrapper : public BaseNode {
   public:
    explicit SpatialNNWrapper(const std::string& daiNodeName,
                              ros::NodeHandle node,
                              std::shared_ptr<dai::Pipeline> pipeline,
                              const dai::CameraBoardSocket& socket = dai::CameraBoardSocket::CAM_A);
    ~SpatialNNWrapper();
    void updateParams(parametersConfig& config) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0);
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    std::unique_ptr<param_handlers::NNParamHandler> ph;
    std::unique_ptr<BaseNode> nnNode;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver