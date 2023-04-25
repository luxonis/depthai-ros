#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace dai {
class Pipeline;
class Device;
class DataOutputQueue;
class ADatatype;
namespace node {
class NeuralNetwork;
class ImageManip;
class XLinkOut;
}  // namespace node
namespace ros {
class ImageConverter;
}
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
class NNParamHandler;
}
namespace dai_nodes {
namespace nn {
class Segmentation : public BaseNode {
   public:
    Segmentation(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline);
    ~Segmentation();
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    cv::Mat decodeDeeplab(cv::Mat mat);
    void segmentationCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    std::vector<std::string> labelNames;
    std::unique_ptr<dai::ros::ImageConverter> imageConverter;
    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager;
    image_transport::CameraPublisher nnPub, ptPub;
    sensor_msgs::msg::CameraInfo nnInfo;
    std::shared_ptr<dai::node::NeuralNetwork> segNode;
    std::shared_ptr<dai::node::ImageManip> imageManip;
    std::unique_ptr<param_handlers::NNParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> nnQ, ptQ;
    std::shared_ptr<dai::node::XLinkOut> xoutNN, xoutPT;
    std::string nnQName, ptQName;
};

}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver