#include <cstdio>
#include <depthai/common/CameraBoardSocket.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <depthai/rtabmap/RTABMapSLAM.hpp>

#include "depthai/basalt/BasaltVIO.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/GridMapConverter.hpp"
#include "depthai_bridge/PointCloudConverter.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_bridge/TransformDataConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

int main(int argc, char** argv) {
    std::string tfPrefix = "oak";
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
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto slam = pipeline.create<dai::node::RTABMapSLAM>();
    std::map<std::string, std::string> params;
    params.insert({"RGBD/CreateOccupancyGrid", "true"});
    params.insert({"Grid/3D", "true"});
    params.insert({"Rtabmap/SaveWMState", "true"});
    slam->setParams(params);

    stereo->setExtendedDisparity(false);
    stereo->setSubpixel(true);
    stereo->setLeftRightCheck(true);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->enableDistortionCorrection(true);
    stereo->initialConfig->setLeftRightCheckThreshold(10);
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::DEFAULT);

    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 200);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 200);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);
    left->requestOutput(std::make_pair(width, height))->link(stereo->left);
    right->requestOutput(std::make_pair(width, height))->link(stereo->right);
    stereo->syncedLeft.link(odom->left);
    stereo->syncedRight.link(odom->right);
    stereo->depth.link(slam->depth);
    stereo->rectifiedLeft.link(slam->rect);
    imu->out.link(odom->imu);

    odom->transform.link(slam->odom);

    // Create output queue
    auto slamQ = slam->odomCorrection.createOutputQueue(8, false);
    auto odomQ = odom->transform.createOutputQueue(8, false);
    auto occupancyGridQ = slam->occupancyGridMap.createOutputQueue(8, false);
    auto obstaclePCLQ = slam->obstaclePCL.createOutputQueue(8, false);
    auto groundPCLQ = slam->groundPCL.createOutputQueue(8, false);
    pipeline.start();

    // Create a bridge publisher for Odom images
    auto slamConv = std::make_shared<depthai_bridge::TransformDataConverter>("map", "odom");
    slamConv->fixQuaternion();
    auto odomConv = std::make_shared<depthai_bridge::TransformDataConverter>("odom", "oak");
    auto mapConv = std::make_shared<depthai_bridge::GridMapConverter>("map");
    auto pclConv = std::make_shared<depthai_bridge::PointCloudConverter>("map");

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

    auto slamPub = std::make_unique<depthai_bridge::BridgePublisher<geometry_msgs::msg::TransformStamped, dai::TransformData>>(
        slamQ,
        node,
        "slam",
        [slamConv](std::shared_ptr<dai::TransformData> msg, std::deque<geometry_msgs::msg::TransformStamped>& rosMsgs) { slamConv->toRosMsg(msg, rosMsgs); },
        30,
        false);

    auto mapPub = std::make_unique<depthai_bridge::BridgePublisher<nav_msgs::msg::OccupancyGrid, dai::MapData>>(
        occupancyGridQ,
        node,
        "map",
        [mapConv](std::shared_ptr<dai::MapData> msg, std::deque<nav_msgs::msg::OccupancyGrid>& rosMsgs) {  //
            mapConv->toRosMsg(msg, rosMsgs);
        },
        30,
        false);

    auto obstaclePub = std::make_unique<depthai_bridge::BridgePublisher<sensor_msgs::msg::PointCloud2, dai::PointCloudData>>(
        obstaclePCLQ,
        node,
        "obstacle",
        [pclConv](std::shared_ptr<dai::PointCloudData> msg, std::deque<sensor_msgs::msg::PointCloud2>& rosMsgs) {  //
            pclConv->toRosMsg(msg, rosMsgs);
        },
        30,
        false);

    auto groundPub = std::make_unique<depthai_bridge::BridgePublisher<sensor_msgs::msg::PointCloud2, dai::PointCloudData>>(
        groundPCLQ,
        node,
        "ground",
        [pclConv](std::shared_ptr<dai::PointCloudData> msg, std::deque<sensor_msgs::msg::PointCloud2>& rosMsgs) {  //
            pclConv->toRosMsg(msg, rosMsgs);
        },
        30,
        false);

    // to publish transform and odom at the same time
    odomPub->enableTransformPub();
    odomPub->addPublisherCallback();
    slamPub->addPublisherCallback();
    mapPub->addPublisherCallback();
    obstaclePub->addPublisherCallback();
    groundPub->addPublisherCallback();

    while(rclcpp::ok() && pipeline.isRunning()) {
        rclcpp::spin(node);
    }

    return 0;
}
