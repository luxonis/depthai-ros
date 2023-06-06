#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace dai {
class Pipeline;
class Device;
class DataInputQueue;
namespace node {
class XLinkIn;
}
namespace ros {
class ImageConverter;
}
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

class SensorWrapper : public BaseNode {
   public:
    explicit SensorWrapper(const std::string& daiNodeName,
                           rclcpp::Node* node,
                           std::shared_ptr<dai::Pipeline> pipeline,
                           std::shared_ptr<dai::Device> device,
                           dai::CameraBoardSocket socket,
                           bool publish = true);
    ~SensorWrapper();
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input in, int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;
    sensor_helpers::ImageSensor getSensorData();

   private:
    void subCB(const sensor_msgs::msg::Image& img);
    std::unique_ptr<BaseNode> sensorNode;
    std::unique_ptr<param_handlers::SensorParamHandler> ph;
    std::unique_ptr<dai::ros::ImageConverter> converter;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
    std::shared_ptr<dai::node::XLinkIn> xIn;
    std::shared_ptr<dai::DataInputQueue> inQ;
    std::string inQName;
    int socketID;
    sensor_helpers::ImageSensor sensorData;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver