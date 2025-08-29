#pragma once

#include <depthai/common/CameraBoardSocket.hpp>

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"

namespace dai {
class Pipeline;
class Device;
class DataInputQueue;
class ADatatype;
namespace node {
class ToF;
class ImageAlign;
}  // namespace node
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
class ToFParamHandler;
}
namespace dai_nodes {

namespace sensor_helpers {
class ImagePublisher;
}  // namespace sensor_helpers
class ToF : public BaseNode {
   public:
    explicit ToF(const std::string& daiNodeName,
                 std::shared_ptr<rclcpp::Node> node,
                 std::shared_ptr<dai::Pipeline> pipeline,
                 const std::string& deviceName,
                 bool rsCompat,
                 dai::CameraBoardSocket boardSocket = dai::CameraBoardSocket::CAM_A);
    ~ToF();
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input& in, int linkType = 0) override;
    void setNames() override;
    void setInOut(std::shared_ptr<dai::Pipeline> pipeline) override;
    dai::Node::Input& getInput(int linkType = 0) override;
    std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> getPublishers() override;
    void closeQueues() override;
    std::shared_ptr<dai::node::ToF> getUnderlyingNode();
    dai::CameraBoardSocket getSocketID();
    dai::CameraBoardSocket getAlignedSocketID();
    bool isAligned();

   private:
    std::shared_ptr<sensor_helpers::ImagePublisher> tofPub;
    std::shared_ptr<dai::node::ToF> tofNode;
    std::shared_ptr<dai::node::ImageAlign> alignNode;
    std::unique_ptr<param_handlers::ToFParamHandler> ph;
    dai::CameraBoardSocket boardSocket;
    dai::CameraBoardSocket alignedSocket;
    std::string tofQName;
    bool aligned;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
