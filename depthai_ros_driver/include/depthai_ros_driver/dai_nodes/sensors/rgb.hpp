#pragma once

#include "depthai_ros_driver/dai_nodes/base_node.hpp"

namespace dai {
class Pipeline;
class Device;
class DataInputQueue;
enum class CameraBoardSocket;
class ADatatype;
namespace node {
class ColorCamera;
class XLinkIn;
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

class RGB : public BaseNode {
   public:
    explicit RGB(const std::string& daiNodeName,
                 std::shared_ptr<rclcpp::Node> node,
                 std::shared_ptr<dai::Pipeline> pipeline,
                 dai::CameraBoardSocket socket,
                 sensor_helpers::ImageSensor sensor,
                 bool publish);
    ~RGB();
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input in, int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;
    std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> getPublishers() override;

   private:
    std::shared_ptr<sensor_helpers::ImagePublisher> rgbPub, previewPub;
    std::shared_ptr<dai::node::ColorCamera> colorCamNode;
    std::unique_ptr<param_handlers::SensorParamHandler> ph;
    std::shared_ptr<dai::DataInputQueue> controlQ;
    std::shared_ptr<dai::node::XLinkIn> xinControl;
    std::string ispQName, previewQName, controlQName;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
