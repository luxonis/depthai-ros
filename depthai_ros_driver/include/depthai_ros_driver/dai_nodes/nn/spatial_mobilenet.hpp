#pragma once

#include <string>

#include "depthai/depthai.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn {

class SpatialMobilenet : public BaseNode {
   public:
    SpatialMobilenet(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline);
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    void mobilenetCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    std::unique_ptr<dai::ros::SpatialDetectionConverter> detConverter;
    std::vector<std::string> labelNames;
    image_transport::CameraPublisher nnPub;
    sensor_msgs::msg::CameraInfo nnInfo;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detPub;
    std::shared_ptr<dai::node::MobileNetSpatialDetectionNetwork> mobileNode;
    std::shared_ptr<dai::node::ImageManip> imageManip;
    std::unique_ptr<param_handlers::NNParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> nnQ;
    std::shared_ptr<dai::node::XLinkOut> xoutNN;
    std::string nnQName;
};

}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver