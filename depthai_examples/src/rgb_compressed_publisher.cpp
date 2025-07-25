#include <cstdio>
#include <functional>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/properties/VideoEncoderProperties.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

int main(int argc, char** argv) {
    int width = 1280;
    int height = 720;
    std::string tfPrefix = "oak";
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rgb_compressed_publisher");

    dai::VideoEncoderProperties::Profile encProfile = static_cast<dai::VideoEncoderProperties::Profile>(
        node->declare_parameter<int>("profile", static_cast<int>(dai::VideoEncoderProperties::Profile::MJPEG)));
    bool publishCompressed = node->declare_parameter<bool>("publish_compressed", false);
    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);

    // Define sources and outputs
    auto rgbCamera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto videoEnc = pipeline.create<dai::node::VideoEncoder>();
    videoEnc->setDefaultProfilePreset(30, encProfile);

    // Create output queue
    auto rgbOut = rgbCamera->requestOutput({width, height}, dai::ImgFrame::Type::NV12);
    rgbOut->link(videoEnc->input);
    auto encQ = videoEnc->out.createOutputQueue(8, false);

    pipeline.start();

    // Create a bridge publisher for RGB images
    auto rgbConverter = std::make_shared<depthai_bridge::ImageConverter>(
        depthai_bridge::getOpticalFrameName(tfPrefix, depthai_bridge::getSocketName(dai::CameraBoardSocket::CAM_A)), false);
    rgbConverter->setUpdateRosBaseTimeOnToRosMsg(false);
    auto calibrationHandler = device->readCalibration();
    auto tfPub =
        std::make_unique<depthai_bridge::TFPublisher>(node, calibrationHandler, device->getConnectedCameraFeatures(), tfPrefix, device->getDeviceName());
    auto rgbCameraInfo = rgbConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, width, height);
    if(!publishCompressed) {
        // We need to pass the type so that image is converted correctly
        rgbConverter->convertFromBitstream(dai::ImgFrame::Type::NV12);
        auto rgbPub = std::make_unique<depthai_bridge::BridgePublisher<sensor_msgs::msg::Image, dai::EncodedFrame>>(
            encQ,
            node,
            "rgb/image",
            [rgbConverter](std::shared_ptr<dai::EncodedFrame> msg, std::deque<sensor_msgs::msg::Image>& rosMsgs) { rgbConverter->toRosMsg(msg, rosMsgs); },
            30,
            rgbCameraInfo,
            "rgb",
            false);

        rgbPub->addPublisherCallback();
        while(rclcpp::ok() && pipeline.isRunning()) {
            rclcpp::spin(node);
        }
    }

    if(encProfile == dai::VideoEncoderProperties::Profile::MJPEG) {
        auto rgbPub = std::make_unique<depthai_bridge::BridgePublisher<sensor_msgs::msg::CompressedImage, dai::EncodedFrame>>(
            encQ,
            node,
            "rgb/image",
            [rgbConverter](std::shared_ptr<dai::EncodedFrame> msg, std::deque<sensor_msgs::msg::CompressedImage>& rosMsgs) {
                rgbConverter->toRosCompressedMsg(msg, rosMsgs);
            },
            30,
            rgbCameraInfo,
            "rgb",
            false);

        rgbPub->addPublisherCallback();
        while(rclcpp::ok() && pipeline.isRunning()) {
            rclcpp::spin(node);
        }
    } else {
        auto rgbPub = std::make_unique<depthai_bridge::BridgePublisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket, dai::EncodedFrame>>(
            encQ,
            node,
            "rgb/image",
            [rgbConverter](std::shared_ptr<dai::EncodedFrame> msg, std::deque<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>& rosMsgs) {
                rgbConverter->toRosFFMPEGPacket(msg, rosMsgs);
            },
            30,
            rgbCameraInfo,
            "rgb",
            false);

        rgbPub->addPublisherCallback();
        while(rclcpp::ok() && pipeline.isRunning()) {
            rclcpp::spin(node);
        }
    }

    return 0;
}
