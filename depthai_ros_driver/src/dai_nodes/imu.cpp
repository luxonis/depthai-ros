#include "depthai_ros_driver/dai_nodes/imu.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
namespace depthai_ros_driver {
namespace dai_nodes {
Imu::Imu(const std::string& dai_node_name, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(dai_node_name, node, pipeline) {
    RCLCPP_INFO(node->get_logger(), "Creating node %s", dai_node_name.c_str());
    set_names();
    imu_node_ = pipeline->create<dai::node::IMU>();
    param_handler_ = std::make_unique<param_handlers::ImuParamHandler>(dai_node_name);
    param_handler_->declareParams(node, imu_node_);
    set_xin_xout(pipeline);
    RCLCPP_INFO(node->get_logger(), "Node %s created", dai_node_name.c_str());
};
void Imu::set_names() {
    imu_q_name_ = getName() + "_imu";
}

void Imu::set_xin_xout(std::shared_ptr<dai::Pipeline> pipeline) {
    xout_imu_ = pipeline->create<dai::node::XLinkOut>();
    xout_imu_->setStreamName(imu_q_name_);
    imu_node_->out.link(xout_imu_->input);
}

void Imu::setupQueues(std::shared_ptr<dai::Device> device) {
    imu_q_ = device->getOutputQueue(imu_q_name_, param_handler_->get_param<int>(getROSNode(), "i_max_q_size"), false);
    imu_q_->addCallback(std::bind(&Imu::imu_q_cb, this, std::placeholders::_1, std::placeholders::_2));
    imu_pub_ = getROSNode()->create_publisher<sensor_msgs::msg::Imu>("~/" + getName() + "/data",10);
}

void Imu::imu_q_cb(const std::string& name, const std::shared_ptr<dai::ADatatype>& data) {
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
        imu_msg.header.frame_id = "camera_link";
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
        imu_pub_->publish(imu_msg);
    }
}

void Imu::link(const dai::Node::Input& in, int /*link_type*/) {
    imu_node_->out.link(in);
}

dai::Node::Input Imu::get_input(int link_type) {
    throw(std::runtime_error("Class Imu has no input."));
}

void Imu::updateParams(const std::vector<rclcpp::Parameter>& params) {
    param_handler_->setRuntimeParams(params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
