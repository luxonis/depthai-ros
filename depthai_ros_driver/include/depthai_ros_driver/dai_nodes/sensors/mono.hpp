#pragma once

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"

namespace dai {
class Pipeline;
class Device;
class DataOutputQueue;
class DataInputQueue;
class ADatatype;
namespace node {
class MonoCamera;
class XLinkIn;
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

namespace camera_info_manager {
class CameraInfoManager;
}

namespace depthai_ros_driver {
namespace param_handlers {
class SensorParamHandler;
}
namespace dai_nodes {

class Mono : public BaseNode {
   public:
    explicit Mono(const std::string& daiNodeName,
                  rclcpp::Node* node,
                  std::shared_ptr<dai::Pipeline> pipeline,
                  dai::CameraBoardSocket socket,
                  sensor_helpers::ImageSensor sensor,
                  bool publish);
    ~Mono();
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input in, int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    std::unique_ptr<dai::ros::ImageConverter> imageConverter;
    image_transport::CameraPublisher monoPub;
    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager;
    std::shared_ptr<dai::node::MonoCamera> monoCamNode;
    std::shared_ptr<dai::node::VideoEncoder> videoEnc;
    std::unique_ptr<param_handlers::SensorParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> monoQ;
    std::shared_ptr<dai::DataInputQueue> controlQ;
    std::shared_ptr<dai::node::XLinkOut> xoutMono;
    std::shared_ptr<dai::node::XLinkIn> xinControl;
    std::string monoQName, controlQName;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver