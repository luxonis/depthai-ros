
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/CameraFeatures.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace dai {
class Pipeline;
class Device;
class DataOutputQueue;
class ADatatype;
class ImgFrame;
namespace node {
class Sync;
class XLinkOut;
class VideoEncoder;
}  // namespace node
namespace ros {
class ImageConverter;
}
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace dai_nodes {
class Sync : public BaseNode {
   public:
    explicit Sync(const std::string& daiNodeName, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<dai::Pipeline> pipeline);
    ~Sync();
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    std::shared_ptr<dai::node::Sync> syncNode;
    std::shared_ptr<dai::node::XLinkOut> xoutFrame;
    void publishOutputs();
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
