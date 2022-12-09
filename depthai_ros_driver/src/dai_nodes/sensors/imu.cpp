#include "depthai_ros_driver/dai_nodes/sensors/imu.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
namespace depthai_ros_driver {
namespace dai_nodes {
Imu::Imu(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_INFO(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    imuNode = pipeline->create<dai::node::IMU>();
    ph = std::make_unique<param_handlers::ImuParamHandler>(daiNodeName);
    ph->declareParams(node, imuNode);
    setXinXout(pipeline);
    RCLCPP_INFO(node->get_logger(), "Node %s created", daiNodeName.c_str());
};
void Imu::setNames() {
    imuQName = getName() + "_imu";
}

void Imu::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutImu = pipeline->create<dai::node::XLinkOut>();
    xoutImu->setStreamName(imuQName);
    imuNode->out.link(xoutImu->input);
}

void Imu::setupQueues(std::shared_ptr<dai::Device> device) {
    imuQ = device->getOutputQueue(imuQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
    auto tfPrefix = std::string(getROSNode()->get_name()) + "_" + getName();
    auto imuMode = static_cast<dai::ros::ImuSyncMethod>(0);
    imuConverter = std::make_unique<dai::ros::ImuConverter>(tfPrefix + "_frame", imuMode, 0.0, 0.0);
    imuQ->addCallback(std::bind(&Imu::imuQCB, this, std::placeholders::_1, std::placeholders::_2));
    imuPub = getROSNode()->create_publisher<sensor_msgs::msg::Imu>("~/" + getName() + "/data", 10);
}

void Imu::closeQueues() {
    imuQ->close();
}

void Imu::imuQCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto imuData = std::dynamic_pointer_cast<dai::IMUData>(data);
    std::deque<sensor_msgs::msg::Imu> deq;
    imuConverter->toRosMsg(imuData, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        imuPub->publish(currMsg);
        deq.pop_front();
    }
}

void Imu::link(const dai::Node::Input& in, int /*linkType*/) {
    imuNode->out.link(in);
}

dai::Node::Input Imu::getInput(int linkType) {
    throw(std::runtime_error("Class Imu has no input."));
}

void Imu::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(getROSNode(), params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
