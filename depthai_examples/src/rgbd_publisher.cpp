#include <cstdio>
#include <functional>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/node/host/RGBD.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/PointCloudConverter.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

int main(int argc, char** argv) {
    std::string tfPrefix = "oak";
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared(tfPrefix);

    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);

    // Define sources and outputs
    auto rgbd = pipeline.create<dai::node::RGBD>()->build(true);

    auto pclQ = rgbd->pcl.createOutputQueue(8, false);

    pipeline.start();

    auto calibrationHandler = device->readCalibration();
    auto tfPub =
        std::make_unique<depthai_bridge::TFPublisher>(node, calibrationHandler, device->getConnectedCameraFeatures(), tfPrefix, device->getDeviceName());
    // Create a bridge publisher for RGB images
    auto pclConverter = std::make_shared<depthai_bridge::PointCloudConverter>(tfPrefix + "_rgb_camera_optical_frame", false);

    pclConverter->setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::METER);
    auto pclPub = std::make_unique<depthai_bridge::BridgePublisher<sensor_msgs::msg::PointCloud2, dai::PointCloudData>>(
        pclQ, node, "points/color", std::bind(&depthai_bridge::PointCloudConverter::toRosMsg, pclConverter, std::placeholders::_1, std::placeholders::_2), 30);

    pclPub->addPublisherCallback();

    while(rclcpp::ok() && pipeline.isRunning()) {
        rclcpp::spin(node);
    }

    return 0;
}
