#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
namespace dai {
class Pipeline;
class Device;
class DataOutputQueue;
class ADatatype;
namespace node {
class StereoDepth;
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
class StereoParamHandler;
}

namespace dai_nodes {
namespace link_types {
enum class StereoLinkType { left, right };
};
class Stereo : public BaseNode {
   public:
    explicit Stereo(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline, std::shared_ptr<dai::Device> device);
    ~Stereo();
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    std::unique_ptr<dai::ros::ImageConverter> imageConverter;
    image_transport::CameraPublisher stereoPub;
    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager;
    std::shared_ptr<dai::node::StereoDepth> stereoCamNode;
    std::shared_ptr<dai::node::VideoEncoder> videoEnc;
    std::unique_ptr<BaseNode> left;
    std::unique_ptr<BaseNode> right;
    std::unique_ptr<param_handlers::StereoParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> stereoQ;
    std::shared_ptr<dai::node::XLinkOut> xoutStereo;
    std::string stereoQName;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver