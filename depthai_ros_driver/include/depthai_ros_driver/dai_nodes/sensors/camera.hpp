#pragma once

#include "depthai_ros_driver/dai_nodes/base_node.hpp"

namespace dai {
class Pipeline;
class Device;
class MessageQueue;
enum class CameraBoardSocket;
class ADatatype;
class InputQueue;
namespace node {
class Camera;
}  // namespace node
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
class SensorParamHandler;
}
namespace dai_nodes {

namespace sensor_helpers {
struct ImageSensor;
class ImagePublisher;
}  // namespace sensor_helpers

class Camera : public BaseNode {
   public:
    explicit Camera(const std::string& daiNodeName,
                    std::shared_ptr<rclcpp::Node> node,
                    std::shared_ptr<dai::Pipeline> pipeline,
                    const std::string& deviceName,
                    bool rsCompat,
                    dai::CameraBoardSocket socket,
                    bool publish);
    ~Camera();
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input in, int linkType = 0) override;
    void setNames() override;
    void setInOut(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;
    std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> getPublishers() override;
    std::shared_ptr<dai::node::Camera> getUnderlyingNode();
    dai::Node::Output* getDefaultOut();

   private:
    std::shared_ptr<sensor_helpers::ImagePublisher> rgbPub, previewPub;
    std::vector<std::string> outputNames;
    std::vector<std::pair<std::string, dai::Node::Output*>> outputs;

    std::shared_ptr<dai::node::Camera> camNode;
    std::unique_ptr<param_handlers::SensorParamHandler> ph;
    std::shared_ptr<dai::InputQueue> controlQ;
    std::string ispQName, previewQName, controlQName;
    dai::Node::Output* defaultOut;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
