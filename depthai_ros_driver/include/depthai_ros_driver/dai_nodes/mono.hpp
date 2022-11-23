#pragma once

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/mono_param_handler.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {

class Mono : public BaseNode {
   public:
    explicit Mono(){};
    virtual ~Mono() = default;
    void initialize(const std::string& dai_node_name, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) override;
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input& in, int link_type = 0) override;
    void set_names(const std::string& dai_node_name) override;
    void set_xin_xout(std::shared_ptr<dai::Pipeline> pipeline) override;

   private:
    void mono_q_cb(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    image_transport::CameraPublisher mono_pub_;
    sensor_msgs::msg::CameraInfo mono_info_;
    std::shared_ptr<dai::node::MonoCamera> mono_cam_node_;
    std::unique_ptr<param_handlers::MonoParamHandler> param_handler_;
    std::shared_ptr<dai::DataOutputQueue> mono_q_;
    std::shared_ptr<dai::DataInputQueue> control_q_;
    std::shared_ptr<dai::node::XLinkOut> xout_mono_;
    std::shared_ptr<dai::node::XLinkIn> xin_control_;
    std::string mono_q_name_, control_q_name_;
};
class MonoFactory : public BaseNodeFactory {
   public:
    std::unique_ptr<BaseNode> create() {
        return std::make_unique<Mono>();
    };
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver