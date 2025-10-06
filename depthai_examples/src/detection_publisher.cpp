#include <cstdio>
#include <functional>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "rclcpp/node.hpp"

int main(int argc, char** argv) {
    int width = 1280;
    int height = 720;
    std::string tfPrefix = "oak";
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared(tfPrefix);

    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);

    auto rgbCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    // Create and configure detection network node
    auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>();

    dai::NNModelDescription modelDescription;
    modelDescription.model = "yolov6-nano";
    detectionNetwork->build(rgbCamera, modelDescription);

    // Create output queues
    auto qRgb = detectionNetwork->passthrough.createOutputQueue();
    auto qDet = detectionNetwork->out.createOutputQueue();

    pipeline.start();

    // Create a bridge publisher for RGB images
    auto rgbConverter = std::make_shared<depthai_bridge::ImageConverter>(tfPrefix + "_rgb_camera_optical_frame", false);

    auto detConverter = std::make_shared<depthai_bridge::ImgDetectionConverter>(tfPrefix + "_rgb_camera_optical_frame");

    auto calibrationHandler = device->readCalibration();
    auto tfPub =
        std::make_unique<depthai_bridge::TFPublisher>(node, calibrationHandler, device->getConnectedCameraFeatures(), tfPrefix, device->getDeviceName());
    auto rgbCameraInfo = rgbConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, width, height);

    auto rgbPub = std::make_unique<depthai_bridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>>(
        qRgb,
        node,
        "rgb/image",
        [rgbConverter](std::shared_ptr<dai::ImgFrame> msg, std::deque<sensor_msgs::msg::Image>& rosMsgs) { rgbConverter->toRosMsg(msg, rosMsgs); },
        30,
        rgbCameraInfo,
        "rgb");

    rgbPub->addPublisherCallback();

    auto detPub = std::make_unique<depthai_bridge::BridgePublisher<vision_msgs::msg::Detection2DArray, dai::ImgDetections>>(
        qDet,
        node,
        "rgb/detections",
        std::bind(&depthai_bridge::ImgDetectionConverter::toRosMsg, detConverter, std::placeholders::_1, std::placeholders::_2),
        30);

    detPub->addPublisherCallback();

    while(rclcpp::ok() && pipeline.isRunning()) {
        rclcpp::spin(node);
    }

    return 0;
}
