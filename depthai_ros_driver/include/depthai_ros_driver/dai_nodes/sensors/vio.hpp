#pragma once

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace dai {
class Pipeline;
class Device;
class ADatatype;
namespace node {
class BasaltVIO;
}  // namespace node
}  // namespace dai

namespace depthai_bridge {
class TransformDataConverter;
};

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp
namespace tf2_ros {
class TransformBroadcaster;
}

namespace depthai_ros_driver {
namespace param_handlers {
class VioParamHandler;
}
namespace dai_nodes {
namespace link_types {
enum class VioLinkType { left, right, imu };
};
class SensorWrapper;
class Stereo;
class Imu;
class Vio : public BaseNode {
   public:
    explicit Vio(const std::string& daiNodeName,
                 std::shared_ptr<rclcpp::Node> node,
                 std::shared_ptr<dai::Pipeline> pipeline,
                 std::shared_ptr<dai::Device> device,
                 bool rsCompat,
                 SensorWrapper& left,
                 SensorWrapper& right,
                 Imu& imu);
    explicit Vio(const std::string& daiNodeName,
                 std::shared_ptr<rclcpp::Node> node,
                 std::shared_ptr<dai::Pipeline> pipeline,
                 std::shared_ptr<dai::Device> device,
                 bool rsCompat,
                 Stereo& stereo,
                 Imu& imu);
    ~Vio();
    void setupQueues(std::shared_ptr<dai::Device> device) override;
    void link(dai::Node::Input& in, int linkType = 0) override;
    dai::Node::Input& getInput(int linkType = 0) override;
    void setNames() override;
    void setInOut(std::shared_ptr<dai::Pipeline> pipeline) override;
    void closeQueues() override;
    std::shared_ptr<dai::node::BasaltVIO> getUnderlyingNode();

   private:
    std::unique_ptr<depthai_bridge::TransformDataConverter> odomConv;
    void transCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data);
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub;
    std::shared_ptr<dai::node::BasaltVIO> vioNode;
    std::unique_ptr<param_handlers::VioParamHandler> ph;
    std::shared_ptr<dai::MessageQueue> transQ;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBr;
    std::string frameId, childFrameId;
    bool publishTf;
    dai::CameraBoardSocket socket;
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
