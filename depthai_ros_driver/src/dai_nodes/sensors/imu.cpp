#include "depthai_ros_driver/dai_nodes/sensors/imu.hpp"

#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_ros_driver/param_handlers/imu_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "depthai_ros_msgs/msg/imu_with_magnetic_field.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
Imu::Imu(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline, std::shared_ptr<dai::Device> device)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    imuNode = pipeline->create<dai::node::IMU>();
    ph = std::make_unique<param_handlers::ImuParamHandler>(node, daiNodeName);
    ph->declareParams(imuNode, device->getConnectedIMU());
    setXinXout(pipeline);
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
}
Imu::~Imu() = default;
void Imu::setNames() {
    imuQName = getName() + "_imu";
}

void Imu::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutImu = pipeline->create<dai::node::XLinkOut>();
    xoutImu->setStreamName(imuQName);
    imuNode->out.link(xoutImu->input);
}

void Imu::setupQueues(std::shared_ptr<dai::Device> device) {
    imuQ = device->getOutputQueue(imuQName, ph->getParam<int>("i_max_q_size"), false);
    auto tfPrefix = std::string(getROSNode()->get_name()) + "_" + getName();
    auto imuMode = ph->getSyncMethod();
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = rclcpp::QosOverridingOptions();
    imuConverter = std::make_unique<dai::ros::ImuConverter>(tfPrefix + "_frame",
                                                            imuMode,
                                                            ph->getParam<float>("i_acc_cov"),
                                                            ph->getParam<float>("i_gyro_cov"),
                                                            ph->getParam<float>("i_rot_cov"),
                                                            ph->getParam<float>("i_mag_cov"),
                                                            ph->getParam<bool>("i_enable_rotation"),
                                                            ph->getParam<bool>("i_get_base_device_timestamp"));
    imuConverter->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
    param_handlers::imu::ImuMsgType msgType = ph->getMsgType();
    switch(msgType) {
        case param_handlers::imu::ImuMsgType::IMU: {
            rosImuPub = getROSNode()->create_publisher<sensor_msgs::msg::Imu>("~/" + getName() + "/data", 10, options);
            imuQ->addCallback(std::bind(&Imu::imuRosQCB, this, std::placeholders::_1, std::placeholders::_2));
            break;
        }
        case param_handlers::imu::ImuMsgType::IMU_WITH_MAG: {
            daiImuPub = getROSNode()->create_publisher<depthai_ros_msgs::msg::ImuWithMagneticField>("~/" + getName() + "/data", 10, options);
            imuQ->addCallback(std::bind(&Imu::imuDaiRosQCB, this, std::placeholders::_1, std::placeholders::_2));
            break;
        }
        case param_handlers::imu::ImuMsgType::IMU_WITH_MAG_SPLIT: {
            rosImuPub = getROSNode()->create_publisher<sensor_msgs::msg::Imu>("~/" + getName() + "/data", 10, options);
            magPub = getROSNode()->create_publisher<sensor_msgs::msg::MagneticField>("~/" + getName() + "/mag", 10, options);
            imuQ->addCallback(std::bind(&Imu::imuMagQCB, this, std::placeholders::_1, std::placeholders::_2));
            break;
        }
        default: {
            break;
        }
    }
}

void Imu::closeQueues() {
    imuQ->close();
}

void Imu::imuRosQCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto imuData = std::dynamic_pointer_cast<dai::IMUData>(data);
    std::deque<sensor_msgs::msg::Imu> deq;
    imuConverter->toRosMsg(imuData, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        rosImuPub->publish(currMsg);
        deq.pop_front();
    }
}
void Imu::imuDaiRosQCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto imuData = std::dynamic_pointer_cast<dai::IMUData>(data);
    std::deque<depthai_ros_msgs::msg::ImuWithMagneticField> deq;
    imuConverter->toRosDaiMsg(imuData, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        daiImuPub->publish(currMsg);
        deq.pop_front();
    }
}
void Imu::imuMagQCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto imuData = std::dynamic_pointer_cast<dai::IMUData>(data);
    std::deque<depthai_ros_msgs::msg::ImuWithMagneticField> deq;
    imuConverter->toRosDaiMsg(imuData, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        sensor_msgs::msg::Imu imu = currMsg.imu;
        sensor_msgs::msg::MagneticField field = currMsg.field;
        imu.header = currMsg.header;
        field.header = currMsg.header;
        rosImuPub->publish(imu);
        magPub->publish(field);
        deq.pop_front();
    }
}
void Imu::link(dai::Node::Input in, int /*linkType*/) {
    imuNode->out.link(in);
}

void Imu::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
