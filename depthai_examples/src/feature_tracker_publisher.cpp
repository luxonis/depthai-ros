#include <cstdio>
#include <functional>

#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_ros_msgs/msg/tracked_features.hpp"
#include "rclcpp/rclcpp.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/FeatureTracker.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/TrackedFeaturesConverter.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("feature_tracker");

    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);


    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, {}, 30);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, {}, 30);
    auto featureTrackerLeft = pipeline.create<dai::node::FeatureTracker>();
    auto featureTrackerRight = pipeline.create<dai::node::FeatureTracker>();

    // Linking
    monoLeft->requestOutput({640, 400})->link(featureTrackerLeft->inputImage);

    monoRight->requestOutput({640, 400})->link(featureTrackerRight->inputImage);

    // By default the least mount of resources are allocated
    // increasing it improves performance when optical flow is enabled
    auto numShaves = 2;
    auto numMemorySlices = 2;
    featureTrackerLeft->setHardwareResources(numShaves, numMemorySlices);
    featureTrackerRight->setHardwareResources(numShaves, numMemorySlices);

    auto featureTrackerConfig = featureTrackerRight->initialConfig.get();

    auto outputFeaturesLeftQueue = featureTrackerLeft->outputFeatures.createOutputQueue(8, false);
    auto outputFeaturesRightQueue = featureTrackerRight->outputFeatures.createOutputQueue(8, false);
    std::string tfPrefix = "oak";
    depthai_bridge::TrackedFeaturesConverter leftConverter(tfPrefix + "_left_camera_optical_frame", true);

    depthai_bridge::TrackedFeaturesConverter rightConverter(tfPrefix + "_right_camera_optical_frame", true);

    pipeline.start();
    auto calibrationHandler = device->readCalibration();
    auto tfPub = std::make_unique<depthai_bridge::TFPublisher>(node, calibrationHandler, device->getConnectedCameraFeatures(), "oak", device->getDeviceName());

    depthai_bridge::BridgePublisher<depthai_ros_msgs::msg::TrackedFeatures, dai::TrackedFeatures> featuresPubL(
        outputFeaturesLeftQueue,
        node,
        "features_left",
        std::bind(&depthai_bridge::TrackedFeaturesConverter::toRosMsg, &leftConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "features_left");

    featuresPubL.addPublisherCallback();

    depthai_bridge::BridgePublisher<depthai_ros_msgs::msg::TrackedFeatures, dai::TrackedFeatures> featuresPubR(
        outputFeaturesRightQueue,
        node,
        "features_right",
        std::bind(&depthai_bridge::TrackedFeaturesConverter::toRosMsg, &rightConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "features_right");

    featuresPubR.addPublisherCallback();
    while(rclcpp::ok() && pipeline.isRunning()) {
        rclcpp::spin(node);
    }

    return 0;
}
