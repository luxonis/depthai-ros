#include <cstdio>
#include <functional>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ToF.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "rclcpp/node.hpp"

int main(int argc, char** argv) {
    int width = 640;
    int height = 480;
    std::string tfPrefix = "oak";
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("tof_publisher");

    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);

    // Define sources and outputs
    auto tofCamera = pipeline.create<dai::node::ToF>()->build();

    // Create output queue
    auto tofOutputQueue = tofCamera->depth.createOutputQueue(8, false);

    pipeline.start();

    // Create a bridge publisher for tof images
    auto tofConverter = std::make_shared<depthai_bridge::ImageConverter>(tfPrefix + "_rgb_camera_optical_frame", false);

    auto calibrationHandler = device->readCalibration();
    auto tfPub =
        std::make_unique<depthai_bridge::TFPublisher>(node, calibrationHandler, device->getConnectedCameraFeatures(), tfPrefix, device->getDeviceName());
    auto tofCameraInfo = tofConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, width, height);

    auto tofPub = std::make_unique<depthai_bridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>>(
        tofOutputQueue,
        node,
        "tof/image",
        std::bind(&depthai_bridge::ImageConverter::toRosMsg, tofConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        tofCameraInfo,
        "tof");

    tofPub->addPublisherCallback();

    while(rclcpp::ok() && pipeline.isRunning()) {
        rclcpp::spin(node);
    }

    return 0;
}
