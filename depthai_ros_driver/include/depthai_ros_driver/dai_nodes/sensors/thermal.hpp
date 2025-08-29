#pragma once

#include "depthai/common/CameraFeatures.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"

namespace dai {
class Pipeline;
class Device;
class ADatatype;
namespace node {
class Thermal;
}  // namespace node
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
class ThermalParamHandler;
}
namespace dai_nodes {

namespace sensor_helpers {
class ImagePublisher;
}  // namespace sensor_helpers
class Thermal : public BaseNode {
   public:
    explicit Thermal(const std::string& daiNodeName,
                     std::shared_ptr<rclcpp::Node> node,
                     std::shared_ptr<dai::Pipeline> pipeline,
                     const std::string& deviceName,
                     bool rsCompat);
    ~Thermal();
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input& in, int linkType = 0) override;
    void setNames() override;
    void setInOut(std::shared_ptr<dai::Pipeline> pipeline) override;
    std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> getPublishers() override;
    void closeQueues() override;

   private:
    std::shared_ptr<sensor_helpers::ImagePublisher> thermalPub, thermalRawPub;
    std::shared_ptr<dai::node::Thermal> thermalNode;
    std::unique_ptr<param_handlers::ThermalParamHandler> ph;
    dai::CameraBoardSocket boardSocket;
    std::shared_ptr<dai::InputQueue> confQ;
    std::string thermalQName, rawQName;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
