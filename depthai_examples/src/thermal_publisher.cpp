#include <cstdio>
#include <functional>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Thermal.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "rclcpp/node.hpp"

int main(int argc, char** argv) {
    int width = 1280;
    int height = 720;
    std::string tfPrefix = "oak";
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared(tfPrefix);

    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);

    // Define sources and outputs
    auto thermalCamera = pipeline.create<dai::node::Thermal>();

    // Create output queue
    auto thermalOutputQueue = thermalCamera->temperature.createOutputQueue(8, false);

    pipeline.start();

    // Create a bridge publisher for thermal images
    auto thermalConverter = std::make_shared<depthai_bridge::ImageConverter>(
        depthai_bridge::getOpticalFrameName(tfPrefix, depthai_bridge::getSocketName(dai::CameraBoardSocket::CAM_A, device->getDeviceName())), false);

    auto calibrationHandler = device->readCalibration();
    auto tfPub =
        std::make_unique<depthai_bridge::TFPublisher>(node, calibrationHandler, device->getConnectedCameraFeatures(), tfPrefix, device->getDeviceName());
    auto thermalCameraInfo = thermalConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, width, height);

    auto thermalPub = std::make_unique<depthai_bridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>>(
        thermalOutputQueue,
        node,
        "thermal/image",
        [thermalConverter](std::shared_ptr<dai::ImgFrame> msg, std::deque<sensor_msgs::msg::Image>& rosMsgs) { thermalConverter->toRosMsg(msg, rosMsgs); },
        30,
        thermalCameraInfo,
        "thermal");

    thermalPub->addPublisherCallback();

    while(rclcpp::ok() && pipeline.isRunning()) {
        rclcpp::spin(node);
    }

    return 0;
}
