#pragma once

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/imu_param_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {

class Imu : public BaseNode {
   public:
    explicit Imu(const std::string& dai_node_name, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline);
    virtual ~Imu() = default;
    void updateParams(const std::vector<rclcpp::Parameter>& params) override;
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(const dai::Node::Input& in, int link_type = 0) override;
    dai::Node::Input get_input(int link_type = 0) override;
    void set_names() override;
    void set_xin_xout(std::shared_ptr<dai::Pipeline> pipeline) override;

   private:
    void imu_q_cb(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::shared_ptr<dai::node::IMU> imu_node_;
    std::unique_ptr<param_handlers::ImuParamHandler> param_handler_;
    std::shared_ptr<dai::DataOutputQueue> imu_q_;
    std::shared_ptr<dai::node::XLinkOut> xout_imu_;
    std::string imu_q_name_;
};
class ImuFactory : public BaseNodeFactory {
   public:
    std::unique_ptr<BaseNode> create(const std::string& dai_node_name, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) {
        return std::make_unique<Imu>(dai_node_name, node, pipeline);
    };
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver