#include <cstdio>
#include <functional>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/ImageAlign.hpp"
#include "depthai/pipeline/node/ToF.hpp"
#include "depthai/pipeline/node/host/RGBD.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/PointCloudConverter.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

int main(int argc, char** argv) {
    int width = 640;
    int height = 480;
    std::string tfPrefix = "oak";
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("tof_publisher");

    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);

    // Define sources and outputs
    auto rgbCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, std::nullopt, 15);
    auto tofCamera = pipeline.create<dai::node::ToF>()->build();
    auto rgbd = pipeline.create<dai::node::RGBD>()->build();
    auto align = pipeline.create<dai::node::ImageAlign>();
    align->setRunOnHost(true);

    // Create output queue
    auto rgbOut = rgbCamera->requestOutput({width, height}, dai::ImgFrame::Type::RGB888i, dai::ImgResizeMode::CROP, std::nullopt, true);
    rgbOut->link(align->inputAlignTo);
    tofCamera->depth.link(align->input);

    rgbOut->link(rgbd->inColor);
    align->outputAligned.link(rgbd->inDepth);

    auto pclQ = rgbd->pcl.createOutputQueue();

    pipeline.start();

    // Create a bridge publisher for tof images
    auto pclConverter = std::make_shared<depthai_bridge::PointCloudConverter>(
        depthai_bridge::getOpticalFrameName(tfPrefix, depthai_bridge::getSocketName(dai::CameraBoardSocket::CAM_C, device->getDeviceName())), false);
    pclConverter->setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::METER);

    auto calibrationHandler = device->readCalibration();
    auto tfPub =
        std::make_unique<depthai_bridge::TFPublisher>(node, calibrationHandler, device->getConnectedCameraFeatures(), tfPrefix, device->getDeviceName());

    auto pclPub = std::make_unique<depthai_bridge::BridgePublisher<sensor_msgs::msg::PointCloud2, dai::PointCloudData>>(
        pclQ,
        node,
        "pcl/data",
        std::bind(&depthai_bridge::PointCloudConverter::toRosMsg, pclConverter, std::placeholders::_1, std::placeholders::_2),
        1,
        "",
        "pcl");

    pclPub->addPublisherCallback();

    while(rclcpp::ok() && pipeline.isRunning()) {
        rclcpp::spin(node);
    }

    return 0;
}
