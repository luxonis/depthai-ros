#pragma once

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "rclcpp/service.hpp"
#include "std_srvs/srv/trigger.hpp"

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
    void triggerStillCB(std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);

    std::shared_ptr<sensor_helpers::ImagePublisher> rgbPub, previewPub, stillPub;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr triggerStillService;
    std::shared_ptr<dai::node::ColorCamera> colorCamNode;
    std::unique_ptr<param_handlers::SensorParamHandler> ph;
    std::shared_ptr<dai::DataInputQueue> controlQ;
    std::shared_ptr<dai::node::XLinkIn> xinControl;
    std::string ispQName, previewQName, controlQName, stillQName;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
