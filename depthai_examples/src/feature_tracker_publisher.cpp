#include <cstdio>
#include <functional>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/FeatureTracker.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_bridge/TrackedFeaturesConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "depthai_ros_msgs/msg/tracked_features.hpp"
#include "rclcpp/node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::string tfPrefix = "oak";
    auto node = rclcpp::Node::make_shared(tfPrefix);

    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, {}, 30);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, {}, 30);
    auto featureTrackerLeft = pipeline.create<dai::node::FeatureTracker>();
    auto featureTrackerRight = pipeline.create<dai::node::FeatureTracker>();

    // Linking
    monoLeft->requestOutput({640, 640}, dai::ImgFrame::Type::GRAY8)->link(featureTrackerLeft->inputImage);

    monoRight->requestOutput({640, 640}, dai::ImgFrame::Type::GRAY8)->link(featureTrackerRight->inputImage);
    featureTrackerLeft->initialConfig->setCornerDetector(dai::FeatureTrackerConfig::CornerDetector::Type::HARRIS);
    featureTrackerLeft->initialConfig->setMotionEstimator(false);
    featureTrackerLeft->initialConfig->setNumTargetFeatures(256);

    auto motionEstimator = dai::FeatureTrackerConfig::MotionEstimator();
    motionEstimator.enable = true;
    featureTrackerLeft->initialConfig->setMotionEstimator(motionEstimator);

    auto cornerDetector = dai::FeatureTrackerConfig::CornerDetector();
    cornerDetector.numMaxFeatures = 256;
    cornerDetector.numTargetFeatures = cornerDetector.numMaxFeatures;
    auto thresholds = dai::FeatureTrackerConfig::CornerDetector::Thresholds();
    thresholds.initialValue = 20000;  // Default value

    cornerDetector.thresholds = thresholds;
    featureTrackerLeft->initialConfig->setCornerDetector(cornerDetector);

    featureTrackerRight->initialConfig->setCornerDetector(dai::FeatureTrackerConfig::CornerDetector::Type::HARRIS);
    featureTrackerRight->initialConfig->setMotionEstimator(false);
    featureTrackerRight->initialConfig->setNumTargetFeatures(256);
    featureTrackerRight->initialConfig->setCornerDetector(cornerDetector);
    featureTrackerRight->initialConfig->setMotionEstimator(motionEstimator);

    // By default the least mount of resources are allocated
    // increasing it improves performance when optical flow is enabled
    auto numShaves = 2;
    auto numMemorySlices = 2;
    featureTrackerLeft->setHardwareResources(numShaves, numMemorySlices);
    featureTrackerRight->setHardwareResources(numShaves, numMemorySlices);

    featureTrackerLeft->initialConfig->setCornerDetector(cornerDetector);
    featureTrackerRight->initialConfig->setCornerDetector(cornerDetector);
    auto featureTrackerConfig = featureTrackerRight->initialConfig.get();

    auto outputFeaturesLeftQueue = featureTrackerLeft->outputFeatures.createOutputQueue(8, false);
    auto outputFeaturesRightQueue = featureTrackerRight->outputFeatures.createOutputQueue(8, false);
    auto leftConverter = std::make_shared<depthai_bridge::TrackedFeaturesConverter>(
        depthai_bridge::getOpticalFrameName(tfPrefix, depthai_bridge::getSocketName(dai::CameraBoardSocket::CAM_B, device->getDeviceName())), true);

    auto rightConverter = std::make_shared<depthai_bridge::TrackedFeaturesConverter>(
        depthai_bridge::getOpticalFrameName(tfPrefix, depthai_bridge::getSocketName(dai::CameraBoardSocket::CAM_C, device->getDeviceName())), true);

    pipeline.start();
    auto calibrationHandler = device->readCalibration();
    auto tfPub = std::make_unique<depthai_bridge::TFPublisher>(node, calibrationHandler, device->getConnectedCameraFeatures(), "oak", device->getDeviceName());

    auto featuresPubL = std::make_unique<depthai_bridge::BridgePublisher<depthai_ros_msgs::msg::TrackedFeatures, dai::TrackedFeatures>>(
        outputFeaturesLeftQueue,
        node,
        "features_left",
        std::bind(&depthai_bridge::TrackedFeaturesConverter::toRosMsg, leftConverter, std::placeholders::_1, std::placeholders::_2),
        30);

    featuresPubL->addPublisherCallback();

    auto featuresPubR = std::make_unique<depthai_bridge::BridgePublisher<depthai_ros_msgs::msg::TrackedFeatures, dai::TrackedFeatures>>(
        outputFeaturesRightQueue,
        node,
        "features_right",
        std::bind(&depthai_bridge::TrackedFeaturesConverter::toRosMsg, rightConverter, std::placeholders::_1, std::placeholders::_2),
        30);

    featuresPubR->addPublisherCallback();
    while(rclcpp::ok() && pipeline.isRunning()) {
        rclcpp::spin(node);
    }

    return 0;
}
