#pragma once

#include <string>

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn_wrappers {
class Mobilenet : public BaseNode {
   public:
    Mobilenet(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline);
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    bool isSpatial;
    void MobilenetCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    std::vector<std::string> labelNames;
    image_transport::CameraPublisher nnPub;
    sensor_msgs::msg::CameraInfo nnInfo;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detPub;
    std::shared_ptr<dai::node::MobileNetDetectionNetwork> mobileNode;
    std::shared_ptr<dai::node::ImageManip> imageManip;
    std::unique_ptr<param_handlers::NNParamHandler> paramHandler;
    std::shared_ptr<dai::DataOutputQueue> nnQ;
    std::shared_ptr<dai::node::XLinkOut> xoutNN;
    std::string nnQName;
};
class MobilenetFactory : public BaseNodeFactory {
   public:
    std::unique_ptr<BaseNode> create(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) {
        return std::make_unique<Mobilenet>(daiNodeName, node, pipeline);
    };
};
}  // namespace nn_wrappers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver