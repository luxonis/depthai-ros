#include "depthai_ros_driver/dai_nodes/sensors/imu.hpp"

#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_ros_driver/param_handlers/imu_param_handler.hpp"
#include "depthai_ros_msgs/ImuWithMagneticField.h"
#include "ros/node_handle.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

namespace depthai_ros_driver {
namespace dai_nodes {
Imu::Imu(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> pipeline, std::shared_ptr<dai::Device> device)
    : BaseNode(daiNodeName, node, pipeline) {
    ROS_DEBUG("Creating node %s", daiNodeName.c_str());
    setNames();
    imuNode = pipeline->create<dai::node::IMU>();
    ph = std::make_unique<param_handlers::ImuParamHandler>(node, daiNodeName);
    ph->declareParams(imuNode, device->getConnectedIMU());
    setXinXout(pipeline);
    ROS_DEBUG("Node %s created", daiNodeName.c_str());
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
    auto tfPrefix = std::string(getROSNode().getNamespace()) + "_" + getName();
    tfPrefix.erase(0, 1);
    auto imuMode = ph->getSyncMethod();
    param_handlers::imu::ImuMsgType msgType = ph->getMsgType();
    bool enableMagn = msgType == param_handlers::imu::ImuMsgType::IMU_WITH_MAG || msgType == param_handlers::imu::ImuMsgType::IMU_WITH_MAG_SPLIT;
    imuConverter = std::make_unique<dai::ros::ImuConverter>(tfPrefix + "_frame",
                                                            imuMode,
                                                            ph->getParam<float>("i_acc_cov"),
                                                            ph->getParam<float>("i_gyro_cov"),
                                                            ph->getParam<float>("i_rot_cov"),
                                                            ph->getParam<float>("i_mag_cov"),
                                                            ph->getParam<bool>("i_enable_rotation"),
                                                            enableMagn,
                                                            ph->getParam<bool>("i_get_base_device_timestamp"));
    imuConverter->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
    switch(msgType) {
        case param_handlers::imu::ImuMsgType::IMU: {
            rosImuPub = getROSNode().advertise<sensor_msgs::Imu>(getName() + "/data", 10);
            imuQ->addCallback(std::bind(&Imu::imuRosQCB, this, std::placeholders::_1, std::placeholders::_2));
            break;
        }
        case param_handlers::imu::ImuMsgType::IMU_WITH_MAG: {
            daiImuPub = getROSNode().advertise<depthai_ros_msgs::ImuWithMagneticField>(getName() + "/data", 10);
            imuQ->addCallback(std::bind(&Imu::imuDaiRosQCB, this, std::placeholders::_1, std::placeholders::_2));
            break;
        }
        case param_handlers::imu::ImuMsgType::IMU_WITH_MAG_SPLIT: {
            rosImuPub = getROSNode().advertise<sensor_msgs::Imu>(getName() + "/data", 10);
            magPub = getROSNode().advertise<sensor_msgs::MagneticField>(getName() + "/mag", 10);
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
    std::deque<sensor_msgs::Imu> deq;
    imuConverter->toRosMsg(imuData, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        rosImuPub.publish(currMsg);
        deq.pop_front();
    }
}

void Imu::imuDaiRosQCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto imuData = std::dynamic_pointer_cast<dai::IMUData>(data);
    std::deque<depthai_ros_msgs::ImuWithMagneticField> deq;
    imuConverter->toRosDaiMsg(imuData, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        daiImuPub.publish(currMsg);
        deq.pop_front();
    }
}

void Imu::imuMagQCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto imuData = std::dynamic_pointer_cast<dai::IMUData>(data);
    std::deque<depthai_ros_msgs::ImuWithMagneticField> deq;
    imuConverter->toRosDaiMsg(imuData, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        sensor_msgs::Imu imu = currMsg.imu;
        sensor_msgs::MagneticField field = currMsg.field;
        imu.header = currMsg.header;
        field.header = currMsg.header;
        rosImuPub.publish(imu);
        magPub.publish(field);
        deq.pop_front();
    }
}

void Imu::link(dai::Node::Input in, int /*linkType*/) {
    imuNode->out.link(in);
}

void Imu::updateParams(parametersConfig& config) {
    ph->setRuntimeParams(config);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
