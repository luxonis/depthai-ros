
#pragma once

#include <depthai-shared/common/CameraFeatures.hpp>
#include <depthai/device/DataQueue.hpp>
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "image_transport/image_transport.hpp"

namespace dai {
class Pipeline;
class Device;
class DataOutputQueue;
class ADatatype;
namespace node {
class Camera;
class XLinkOut;
}  // namespace node
}  // namespace dai

namespace camera_info_manager {
class CameraInfoManager;
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

namespace sensor_helpers {
class ImagePublisher;
}  // namespace sensor_helpers
class Thermal : public BaseNode {
   public:
    explicit Thermal(const std::string& daiNodeName,
                 std::shared_ptr<rclcpp::Node> node,
                 std::shared_ptr<dai::Pipeline> pipeline,
                 dai::CameraFeatures camFeatures);
    ~Thermal();
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input in, int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> getPublishers() override;
    void closeQueues() override;

   private:
    void thermalRawCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    std::shared_ptr<sensor_helpers::ImagePublisher> thermalPub;
    std::shared_ptr<dai::node::Camera> camNode;
    std::unique_ptr<param_handlers::SensorParamHandler> ph;
    image_transport::CameraPublisher rawPub;
    dai::CameraBoardSocket boardSocket;
    sensor_msgs::msg::CameraInfo rawInfo;
    std::shared_ptr<dai::node::XLinkOut> xoutRaw;
    std::shared_ptr<dai::DataOutputQueue> rawQ;
    std::string thermalQName, rawQName;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
