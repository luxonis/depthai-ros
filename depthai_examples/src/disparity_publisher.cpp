#include <cstdio>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "rclcpp/node.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::string tfPrefix = "oak";
    auto node = rclcpp::Node::make_shared(tfPrefix);

    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);

    auto stereo = pipeline.create<dai::node::StereoDepth>()->build(true);

    auto dispQ = stereo->disparity.createOutputQueue(8, false);

    auto dispConv = std::make_shared<depthai_bridge::DisparityConverter>(
        depthai_bridge::getOpticalFrameName(tfPrefix, depthai_bridge::getSocketName(dai::CameraBoardSocket::CAM_C, device->getDeviceName())),
        true,
        880,
        7.5,
        20,
        2000);

    pipeline.start();
    auto calibrationHandler = device->readCalibration();
    auto tfPub = std::make_unique<depthai_bridge::TFPublisher>(node, calibrationHandler, device->getConnectedCameraFeatures(), "oak", device->getDeviceName());

    auto dispPub = std::make_unique<depthai_bridge::BridgePublisher<stereo_msgs::msg::DisparityImage, dai::ImgFrame>>(
        dispQ,
        node,
        "disparity/image",
        std::bind(&depthai_bridge::DisparityConverter::toRosMsg, dispConv, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "disparity");

    dispPub->addPublisherCallback();

    while(rclcpp::ok() && pipeline.isRunning()) {
        rclcpp::spin(node);
    }

    return 0;
}
