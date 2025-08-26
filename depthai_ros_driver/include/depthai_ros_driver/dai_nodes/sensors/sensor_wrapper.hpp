#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace dai {
class Pipeline;
class Device;
class MessageQueue;
namespace node {
class Camera;
}
}  // namespace dai
namespace depthai_bridge {
class ImageConverter;
}

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
class SensorParamHandler;
}
namespace dai_nodes {

class Camera;

class SensorWrapper : public BaseNode {
   public:
    explicit SensorWrapper(const std::string& daiNodeName,
                           std::shared_ptr<rclcpp::Node> node,
                           std::shared_ptr<dai::Pipeline> pipeline,
                           const std::string& deviceName,
                           bool rsCompat,
                           dai::CameraBoardSocket socket,
                           bool publish = true);
    ~SensorWrapper();
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input& in, int linkType = 0) override;
    void setNames() override;
    void setInOut(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;
    std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> getPublishers() override;
    std::shared_ptr<dai::node::Camera> getUnderlyingNode();
    dai::Node::Output* getDefaultOut();
    dai::CameraBoardSocket getSocketID();

   private:
    void subCB(const sensor_msgs::msg::Image& img);
    std::unique_ptr<Camera> sensorNode;
    std::unique_ptr<BaseNode> featureTrackerNode, nnNode;
    std::unique_ptr<param_handlers::SensorParamHandler> ph;
    std::unique_ptr<depthai_bridge::ImageConverter> converter;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
    std::shared_ptr<dai::MessageQueue> inQ;
    std::string inQName;
    dai::CameraBoardSocket socketID;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
