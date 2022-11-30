#pragma once

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace depthai_ros_driver {
namespace daiNodes {

namespace linkTypes {
enum class NNLinkType { input, inputDepth };
};

class NN : public BaseNode {
   public:
    explicit NN(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline);
    virtual ~NN() = default;
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int linkType = 0) override;
    dai::Node::Input getInput(int linkType = 0) override;
    void setNames() override;
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override;

   private:
    cv::Mat decodeDeeplab(cv::Mat mat);
    void nnQCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    image_transport::CameraPublisher nnPub;
    sensor_msgs::msg::CameraInfo nnInfo;
    std::shared_ptr<dai::node::NeuralNetwork> nnNode;
    std::shared_ptr<dai::node::ImageManip> imageManip;
    std::unique_ptr<paramHandlers::NNParamHandler> paramHandler;
    std::shared_ptr<dai::DataOutputQueue> nnQ;
    std::shared_ptr<dai::node::XLinkOut> xoutNN;
    std::string nnQName;
};
class NNFactory : public BaseNodeFactory {
   public:
    std::unique_ptr<BaseNode> create(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) {
        return std::make_unique<NN>(daiNodeName, node, pipeline);
    };
};
}  // namespace daiNodes
}  // namespace depthai_ros_driver