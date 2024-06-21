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
class Camera;
class ToF;
class ImageAlign;
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
class ToFParamHandler;
}
namespace dai_nodes {

class ToF : public BaseNode {
   public:
    explicit ToF(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline);
    ~ToF();
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input in, int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    std::unique_ptr<dai::ros::ImageConverter> imageConverter;
    image_transport::CameraPublisher tofPub;
    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager;
    std::shared_ptr<dai::node::Camera> camNode;
    std::shared_ptr<dai::node::ToF> tofNode;
    std::shared_ptr<dai::node::ImageAlign> alignNode;
    std::unique_ptr<param_handlers::ToFParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> tofQ;
    std::shared_ptr<dai::node::XLinkOut> xoutTof;
    std::string tofQName;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver