#include "depthai_ros_driver/dai_nodes/vio/vio.hpp"

#include "depthai/basalt/BasaltVIO.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
namespace depthai_ros_driver {

namespace dai_nodes {

Vio::Vio(const std::string& daiNodeName, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
    vioNode = pipeline->create<dai::node::BasaltVIO>();

    tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    vioOdometryPub = node->create_publisher<nav_msgs::msg::Odometry>("vio/odometry", 10);
    setNames();
}

Vio::~Vio() {}

void Vio::setNames() {
    vioQName = getName() + "_vio";
}
void Vio::publishVioOdometry(const std::string& name, const std::shared_ptr<dai::ADatatype>& msg) {
    auto transData = std::dynamic_pointer_cast<dai::TransformData>(msg);
    auto odometry = nav_msgs::msg::Odometry();
    odometry.header.stamp = getROSNode()->get_clock()->now();
    odometry.header.frame_id = "odom";
    odometry.child_frame_id = "oak-d-base-frame";
    auto trans = transData->getTranslation();
    auto quat = transData->getQuaternion();
    odometry.pose.pose.position.x = trans.x;
    odometry.pose.pose.position.y = trans.y;
    odometry.pose.pose.position.z = trans.z;
    odometry.pose.pose.orientation.x = quat.qx;
    odometry.pose.pose.orientation.y = quat.qy;
    odometry.pose.pose.orientation.z = quat.qz;
    odometry.pose.pose.orientation.w = quat.qw;

    vioOdometryPub->publish(odometry);
    auto tf = geometry_msgs::msg::TransformStamped();
    tf.header.stamp = odometry.header.stamp;
    tf.header.frame_id = odometry.header.frame_id;
    tf.child_frame_id = odometry.child_frame_id;
    tf.transform.translation.x = odometry.pose.pose.position.x;
    tf.transform.translation.y = odometry.pose.pose.position.y;
    tf.transform.translation.z = odometry.pose.pose.position.z;
    tf.transform.rotation = odometry.pose.pose.orientation;
    tfBroadcaster->sendTransform(tf);
}
void Vio::setupQueues(std::shared_ptr<dai::Device> /*device*/) {
    vioQ = vioNode->transform.createOutputQueue(1, false);
    vioQ->addCallback(std::bind(&Vio::publishVioOdometry, this, std::placeholders::_1, std::placeholders::_2));
}

dai::Node::Input& Vio::getInput(int linkType) {
    auto type = static_cast<link_types::VioInputType>(linkType);
    switch(type) {
        case link_types::VioInputType::LEFT:
            return vioNode->left;
        case link_types::VioInputType::RIGHT:
            return vioNode->right;
        case link_types::VioInputType::IMU:
            return vioNode->imu;
        default:
            throw std::runtime_error("Vio::getInput: Invalid linkType");
    }
}
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
