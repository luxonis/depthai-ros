#pragma once

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/stereo_param_handler.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace link_types {
enum class StereoLinkType { left, right };
};
class Stereo : public BaseNode {
   public:
    explicit Stereo(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline, std::shared_ptr<dai::Device> device);
    virtual ~Stereo() = default;
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    void stereoQCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    image_transport::CameraPublisher stereoPub;
    sensor_msgs::msg::CameraInfo stereoInfo;
    std::shared_ptr<dai::node::StereoDepth> stereoCamNode;
    std::unique_ptr<BaseNode> left;
    std::unique_ptr<BaseNode> right;
    std::unique_ptr<param_handlers::StereoParamHandler> paramHandler;
    std::shared_ptr<dai::DataOutputQueue> stereoQ;
    std::shared_ptr<dai::DataInputQueue> controlQ;
    std::shared_ptr<dai::node::XLinkOut> xoutStereo;
    std::shared_ptr<dai::node::XLinkIn> xinControl;
    std::string stereoQName;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver