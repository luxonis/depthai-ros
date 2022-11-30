#pragma once

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/mono_param_handler.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace depthai_ros_driver {
namespace daiNodes {

class Mono : public BaseNode {
   public:
    explicit Mono(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline);
    virtual ~Mono() = default;
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;

   private:
    void monoQCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    image_transport::CameraPublisher monoPub;
    sensor_msgs::msg::CameraInfo monoInfo;
    std::shared_ptr<dai::node::MonoCamera> monoCamNode;
    std::unique_ptr<paramHandlers::MonoParamHandler> paramHandler;
    std::shared_ptr<dai::DataOutputQueue> monoQ;
    std::shared_ptr<dai::DataInputQueue> controlQ;
    std::shared_ptr<dai::node::XLinkOut> xoutMono;
    std::shared_ptr<dai::node::XLinkIn> xinControl;
    std::string monoQName, controlQName;
};
class MonoFactory : public BaseNodeFactory {
   public:
    std::unique_ptr<BaseNode> create(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) {
        return std::make_unique<Mono>(daiNodeName, node, pipeline);
    };
};
}  // namespace daiNodes
}  // namespace depthai_ros_driver