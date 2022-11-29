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
enum class SpatialDetectionLinkType { input, inputDepth };
};

class SpatialDetection : public BaseNode {
   public:
    explicit SpatialDetection(const std::string& dai_node_name, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline);
    virtual ~SpatialDetection() = default;
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int link_type = 0) override;
    dai::Node::Input get_input(int link_type = 0) override;
    void set_names() override;
    void set_xin_xout(std::shared_ptr<dai::Pipeline> pipeline) override;

   private:
    cv::Mat decode_deeplab(cv::Mat mat);
    void SpatialDetection_q_cb(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    image_transport::CameraPublisher spatial_detection_pub_;
    sensor_msgs::msg::CameraInfo spatial_detection_info_;
    std::shared_ptr<dai::node::SpatialDetectionNetwork> spatial_detection_node_;
    std::unique_ptr<param_handlers::NNParamHandler> param_handler_;
    std::shared_ptr<dai::DataOutputQueue> spatial_detection_q_;
    std::shared_ptr<dai::node::XLinkOut> xout_spatial_detection_;
    std::string spatial_detection_q_name_;
};
class SpatialDetectionFactory : public BaseNodeFactory {
   public:
    std::unique_ptr<BaseNode> create(const std::string& dai_node_name, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) {
        return std::make_unique<SpatialDetection>(dai_node_name, node, pipeline);
    };
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver