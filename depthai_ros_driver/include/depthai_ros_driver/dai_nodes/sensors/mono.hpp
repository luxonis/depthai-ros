#pragma once

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/depthai.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/mono_param_handler.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {

class Mono : public BaseNode {
   public:
    explicit Mono(const std::string& daiNodeName,
                  rclcpp::Node* node,
                  std::shared_ptr<dai::Pipeline> pipeline,
                  dai::CameraBoardSocket socket,
                  sensor_helpers::ImageSensor sensor,
                  bool publish);
    virtual ~Mono() = default;
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    std::unique_ptr<dai::ros::ImageConverter> imageConverter;
    image_transport::CameraPublisher monoPub;
    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager;
    std::shared_ptr<dai::node::MonoCamera> monoCamNode;
    std::shared_ptr<dai::node::VideoEncoder> videoEnc;
    std::unique_ptr<param_handlers::MonoParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> monoQ;
    std::shared_ptr<dai::DataInputQueue> controlQ;
    std::shared_ptr<dai::node::XLinkOut> xoutMono;
    std::shared_ptr<dai::node::XLinkIn> xinControl;
    std::string monoQName, controlQName;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver