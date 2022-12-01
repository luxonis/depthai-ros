#pragma once

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {

namespace link_types {
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
    virtual void setNames();
    virtual void setXinXout(std::shared_ptr<dai::Pipeline> pipeline);

   private:
    std::unique_ptr<param_handlers::NNParamHandler> paramHandler;
    std::unique_ptr<BaseNode> nnNode;

    
};
class NNFactory : public BaseNodeFactory {
   public:
    std::unique_ptr<BaseNode> create(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) {
        return std::make_unique<NN>(daiNodeName, node, pipeline);
    };
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver