
#pragma once
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/publisher.hpp"
#include "tf2_ros/transform_broadcaster.h"
namespace dai {
class Pipeline;
class Device;
class MessageQueue;
class ADatatype;
namespace node {
class BasaltVIO;
}  // namespace node
}  // namespace dai
namespace rclcpp {
class Node;
class TimerBase;
}  // namespace rclcpp

namespace depthai_ros_driver {

namespace dai_nodes {

namespace link_types {
enum class VioInputType {
    LEFT,
    RIGHT,
    IMU,
};
}

class Vio : public BaseNode {
   public:
    Vio(const std::string& daiNodeName, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<dai::Pipeline> pipeline);
    ~Vio();
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void setNames() override;
    void publishVioOdometry(const std::string& name, const std::shared_ptr<dai::ADatatype>& msg);
    dai::Node::Input& getInput(int linkType = 0) override;

   private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr vioOdometryPub;
    std::shared_ptr<dai::node::BasaltVIO> vioNode;
    std::shared_ptr<dai::MessageQueue> vioQ;
    std::string vioQName;
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
