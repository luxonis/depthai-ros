#include "depthai_ros_driver/dai_nodes/sensors/imu.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"
namespace depthai_ros_driver {
namespace dai_nodes {
Imu::Imu(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
    ROS_DEBUG("Creating node %s", daiNodeName.c_str());
    setNames();
    imuNode = pipeline->create<dai::node::IMU>();
    ph = std::make_unique<param_handlers::ImuParamHandler>(daiNodeName);
    ph->declareParams(node, imuNode);
    setXinXout(pipeline);
    ROS_DEBUG("Node %s created", daiNodeName.c_str());
}
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
    auto tfPrefix = std::string(getROSNode().getNamespace()) + "_" + getName();
    tfPrefix.erase(0, 1);
    auto imuMode = static_cast<dai::ros::ImuSyncMethod>(0);
    imuConverter = std::make_unique<dai::ros::ImuConverter>(tfPrefix + "_frame", imuMode, 0.0, 0.0);
    imuQ->addCallback(std::bind(&Imu::imuQCB, this, std::placeholders::_1, std::placeholders::_2));
    imuPub = getROSNode().advertise<sensor_msgs::Imu>(getName() + "/data", 10);
}

void Imu::closeQueues() {
    imuQ->close();
}

void Imu::imuQCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto imuData = std::dynamic_pointer_cast<dai::IMUData>(data);
    std::deque<sensor_msgs::Imu> deq;
    imuConverter->toRosMsg(imuData, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        imuPub.publish(currMsg);
        deq.pop_front();
    }
}

void Imu::link(const dai::Node::Input& in, int /*linkType*/) {
    imuNode->out.link(in);
}

void Imu::updateParams(parametersConfig& config) {
    ph->setRuntimeParams(getROSNode(), config);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
