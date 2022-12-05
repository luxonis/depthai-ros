#include "depthai_ros_driver/dai_nodes/imu.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
namespace depthai_ros_driver {
namespace dai_nodes {
Imu::Imu(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_INFO(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    imuNode = pipeline->create<dai::node::IMU>();
    paramHandler = std::make_unique<param_handlers::ImuParamHandler>(daiNodeName);
    paramHandler->declareParams(node, imuNode);
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
    imuQ = device->getOutputQueue(imuQName, paramHandler->get_param<int>(getROSNode(), "i_max_q_size"), false);
    imuQ->addCallback(std::bind(&Imu::imuQCB, this, std::placeholders::_1, std::placeholders::_2));
    imuPub = getROSNode()->create_publisher<sensor_msgs::msg::Imu>("~/" + getName() + "/data", 10);
}

void Imu::closeQueues() {
    imuQ->close();
}

void Imu::imuQCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data) {
    auto imu_data = std::dynamic_pointer_cast<dai::IMUData>(data);
    auto packets = imu_data->packets;
    for(const auto& packet : packets) {
        auto accel = packet.acceleroMeter;
        auto gyro = packet.gyroscope;
        auto rot = packet.rotationVector;
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.linear_acceleration.x = accel.x;
        imu_msg.linear_acceleration.y = accel.y;
        imu_msg.linear_acceleration.z = accel.z;
        imu_msg.angular_velocity.x = gyro.x;
        imu_msg.angular_velocity.y = gyro.y;
        imu_msg.angular_velocity.z = gyro.z;
        imu_msg.header.frame_id = std::string(getROSNode()->get_name()) + "_" + getName() + "_frame";
        imu_msg.header.stamp = getROSNode()->get_clock()->now();
        imu_msg.orientation.x = rot.i;
        imu_msg.orientation.y = rot.j;
        imu_msg.orientation.z = rot.k;
        imu_msg.orientation.w = rot.real;
        // this is based on covariances from BNO055, here we have BNO086, but I'm
        // not sure whether those will differ
        // Source:
        // https://github.com/Vijfendertig/rosserial_adafruit_bno055/blob/532b63db9b0e5e5e9217bd89905001fe979df3a4/src/imu_publisher/imu_publisher.cpp#L42.
        for(unsigned row = 0; row < 3; ++row) {
            for(unsigned col = 0; col < 3; ++col) {
                imu_msg.orientation_covariance[row * 3 + col] = (row == col ? 0.002 : 0.);         // +-2.5deg
                imu_msg.angular_velocity_covariance[row * 3 + col] = (row == col ? 0.003 : 0.);    // +-3deg/s
                imu_msg.linear_acceleration_covariance[row * 3 + col] = (row == col ? 0.60 : 0.);  // +-80mg
            }
        }
        imuPub->publish(imu_msg);
    }
}

void Imu::link(const dai::Node::Input& in, int /*linkType*/) {
    imuNode->out.link(in);
}

dai::Node::Input Imu::getInput(int linkType) {
    throw(std::runtime_error("Class Imu has no input."));
}

void Imu::updateParams(const std::vector<rclcpp::Parameter>& params) {
    paramHandler->setRuntimeParams(getROSNode(), params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
