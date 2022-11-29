#pragma once

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {

namespace link_types {
enum class NNLinkType { input, inputDepth };
};

class NN : public BaseNode {
   public:
    explicit NN(const std::string& dai_node_name, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline);
    virtual ~NN() = default;
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int link_type = 0) override;
    dai::Node::Input get_input(int link_type = 0) override;
    void set_names() override;
    void set_xin_xout(std::shared_ptr<dai::Pipeline> pipeline) override;

   private:
    cv::Mat decode_deeplab(cv::Mat mat);
    void nn_q_cb(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    image_transport::CameraPublisher nn_pub_;
    sensor_msgs::msg::CameraInfo nn_info_;
    std::shared_ptr<dai::node::NeuralNetwork> nn_node_;
    std::shared_ptr<dai::node::ImageManip> image_manip_;
    std::unique_ptr<param_handlers::NNParamHandler> param_handler_;
    std::shared_ptr<dai::DataOutputQueue> nn_q_;
    std::shared_ptr<dai::node::XLinkOut> xout_nn_;
    std::string nn_q_name_;
};
class NNFactory : public BaseNodeFactory {
   public:
    std::unique_ptr<BaseNode> create(const std::string& dai_node_name, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) {
        return std::make_unique<NN>(dai_node_name, node, pipeline);
    };
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver