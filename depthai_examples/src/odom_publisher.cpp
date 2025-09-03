#include <cstdio>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/basalt/BasaltVIO.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/OdomConverter.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "rclcpp/node.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

int main(int argc, char** argv) {
    std::string tfPrefix = "oak";
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("odom_publisher");
    // create tfBroadcaster
    auto tfBr = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);

    // Define sources and outputs
    int fps = 60;
    int width = 640;
    int height = 400;
    // Define sources and outputs
    auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, fps);
    auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, fps);
    auto imu = pipeline.create<dai::node::IMU>();
    auto odom = pipeline.create<dai::node::BasaltVIO>();


    // Linking
    left->requestOutput(std::make_pair(width, height))->link(odom->left);
    right->requestOutput(std::make_pair(width, height))->link(odom->right);
    imu->out.link(odom->imu);
    // Enable ACCELEROMETER_RAW at 480 hz rate
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 480);
    // Enable GYROSCOPE_RAW at 400 hz rate
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    // Set batch report threshold and max batch reports
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);

    // Create output queue
    auto odomQ = odom->transform.createOutputQueue(8, false);
    pipeline.start();

    // Create a bridge publisher for RGB images
    auto odomConv = std::make_shared<depthai_bridge::OdomConverter>(depthai_bridge::getFrameName(tfPrefix, "imu_frame"));

    auto calibrationHandler = device->readCalibration();
    auto tfPub =
        std::make_unique<depthai_bridge::TFPublisher>(node, calibrationHandler, device->getConnectedCameraFeatures(), tfPrefix, device->getDeviceName());

    // auto odomPub = std::make_unique<depthai_bridge::BridgePublisher<nav_msgs::msg::Odometry, dai::TransformData>>(
    //     odomQ,
    //     node,
    //     "odom",
    //     [odomConv, tfBr](std::shared_ptr<dai::TransformData> msg, std::deque<nav_msgs::msg::Odometry>& rosMsgs) { 
    //         odomConv->toRosMsg(msg, rosMsgs);
    //         // create transform msg
    //         auto transformMsg = std::make_shared<geometry_msgs::msg::TransformStamped>();
    //         transformMsg->header.stamp = rosMsgs.back().header.stamp;
    //         transformMsg->header.frame_id = "odom";
    //         transformMsg->child_frame_id = "oak";
    //         transformMsg->transform.translation.x = rosMsgs.back().pose.pose.position.x;
    //         transformMsg->transform.translation.y = rosMsgs.back().pose.pose.position.y;
    //         transformMsg->transform.translation.z = rosMsgs.back().pose.pose.position.z;
    //         transformMsg->transform.rotation = rosMsgs.back().pose.pose.orientation;
    //         // publish transform
    //         tfBr->sendTransform(*transformMsg);
    //
    //     },
    //     30,
    //     "",
    //     "");
    //
    // odomPub->addPublisherCallback();

    while(rclcpp::ok() && pipeline.isRunning()) {
        rclcpp::spin_some(node);

        auto odomMsg = odomQ->get<dai::TransformData>();
        auto transformMsg = std::make_shared<geometry_msgs::msg::TransformStamped>();
        transformMsg->header.stamp = node->now();
        transformMsg->header.frame_id = "odom";
        transformMsg->child_frame_id = "oak";
        auto translation = odomMsg->getTranslation();
        auto quat = odomMsg->getQuaternion();
        transformMsg->transform.translation.x = translation.x;
        transformMsg->transform.translation.y = translation.y;
        transformMsg->transform.translation.z = translation.z;
        transformMsg->transform.rotation.x = quat.qx;
        transformMsg->transform.rotation.y = quat.qy;
        transformMsg->transform.rotation.z = quat.qz;
        transformMsg->transform.rotation.w = quat.qw;
        tfBr->sendTransform(*transformMsg);
    }

    return 0;
}
