#include <cstdio>

#include "depthai/basalt/BasaltVIO.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_bridge/TransformDataConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "rclcpp/node.hpp"

int main(int argc, char** argv) {
    std::string tfPrefix = "odom";
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared(tfPrefix);
    auto tfBr = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);

    int fps = 60;
    int width = 640;
    int height = 400;
    auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, fps);
    auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, std::nullopt, fps);
    auto imu = pipeline.create<dai::node::IMU>();
    auto odom = pipeline.create<dai::node::BasaltVIO>();

    left->requestOutput(std::make_pair(width, height))->link(odom->left);
    right->requestOutput(std::make_pair(width, height))->link(odom->right);
    imu->out.link(odom->imu);
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 480);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);

    // Create output queue
    auto odomQ = odom->transform.createOutputQueue(8, false);
    pipeline.start();

    // Create a bridge publisher for Odom images
    auto odomConv = std::make_shared<depthai_bridge::TransformDataConverter>(tfPrefix, "oak");

    auto calibrationHandler = device->readCalibration();
    auto tfPub =
        std::make_unique<depthai_bridge::TFPublisher>(node, calibrationHandler, device->getConnectedCameraFeatures(), tfPrefix, device->getDeviceName());

    auto odomPub = std::make_unique<depthai_bridge::BridgePublisher<nav_msgs::msg::Odometry, dai::TransformData>>(
        odomQ,
        node,
        "odom",
        [odomConv](std::shared_ptr<dai::TransformData> msg, std::deque<nav_msgs::msg::Odometry>& rosMsgs) { odomConv->toRosMsg(msg, rosMsgs); },
        30,
        false);

    odomPub->enableTransformPub();
    odomPub->addPublisherCallback();

    while(rclcpp::ok() && pipeline.isRunning()) {
        rclcpp::spin(node);
    }

    return 0;
}
