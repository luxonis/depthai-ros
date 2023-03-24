#pragma once

#include <memory>
#include <string>
#include <vector>

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "rclcpp/publisher.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace dai {
class Pipeline;
class Device;
class DataOutputQueue;
class ADatatype;
namespace node {
class MobileNetDetectionNetwork;
class ImageManip;
class XLinkOut;
}  // namespace node
namespace ros {
class ImgDetectionConverter;
}

}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace dai {
class Pipeline;
class Device;
class DataOutputQueue;
class ADatatype;
namespace node {
class MobileNetDetectionNetwork;
class ImageManip;
class XLinkOut;
}  // namespace node
namespace ros {
class ImgDetectionConverter;
}

}  // namespace dai

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
class Mobilenet : public BaseNode {
   public:
    Mobilenet(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline);
    ~Mobilenet();
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;

   private:
    void mobilenetCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    std::unique_ptr<dai::ros::ImgDetectionConverter> detConverter;
    std::vector<std::string> labelNames;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detPub;
    std::shared_ptr<dai::node::MobileNetDetectionNetwork> mobileNode;
    std::shared_ptr<dai::node::ImageManip> imageManip;
    std::unique_ptr<param_handlers::NNParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> nnQ;
    std::shared_ptr<dai::node::XLinkOut> xoutNN;
    std::string nnQName;
};

}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver