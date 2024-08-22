
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai_ros_driver/dai_nodes/base_node.hpp"

namespace dai {
class Pipeline;
class Device;
class DataOutputQueue;
class ADatatype;
class ImgFrame;
namespace node {
class Sync;
class XLinkOut;
}  // namespace node
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
class SyncParamHandler;
}
namespace dai_nodes {
class Sync : public BaseNode {
   public:
    explicit Sync(const std::string& daiNodeName, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<dai::Pipeline> pipeline);
    ~Sync();
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input in, int linkType = 0) override;
    dai::Node::Input getInputByName(const std::string& name = "") override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;
    void addPublishers(const std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>>& pubs);

   private:
    std::unique_ptr<param_handlers::SyncParamHandler> paramHandler;
    std::shared_ptr<dai::node::Sync> syncNode;
    std::string syncOutputName;
    std::shared_ptr<dai::node::XLinkOut> xoutFrame;
    std::shared_ptr<dai::DataOutputQueue> outQueue;
    void publishOutputs();
    std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> publishers;
    std::vector<std::string> syncNames;
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
