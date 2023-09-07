#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

#include "depthai_ros_msgs/TrackedFeatures.h"
#include "ros/ros.h"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/FeatureTracker.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/TrackedFeaturesConverter.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "feature_tracker_node");
    ros::NodeHandle pnh("~");

    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto featureTrackerLeft = pipeline.create<dai::node::FeatureTracker>();
    auto featureTrackerRight = pipeline.create<dai::node::FeatureTracker>();

    auto xoutTrackedFeaturesLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutTrackedFeaturesRight = pipeline.create<dai::node::XLinkOut>();

    xoutTrackedFeaturesLeft->setStreamName("trackedFeaturesLeft");
    xoutTrackedFeaturesRight->setStreamName("trackedFeaturesRight");

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setCamera("left");
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setCamera("right");

    // Linking
    monoLeft->out.link(featureTrackerLeft->inputImage);
    featureTrackerLeft->outputFeatures.link(xoutTrackedFeaturesLeft->input);

    monoRight->out.link(featureTrackerRight->inputImage);
    featureTrackerRight->outputFeatures.link(xoutTrackedFeaturesRight->input);

    // By default the least mount of resources are allocated
    // increasing it improves performance when optical flow is enabled
    auto numShaves = 2;
    auto numMemorySlices = 2;
    featureTrackerLeft->setHardwareResources(numShaves, numMemorySlices);
    featureTrackerRight->setHardwareResources(numShaves, numMemorySlices);

    auto featureTrackerConfig = featureTrackerRight->initialConfig.get();

    std::shared_ptr<dai::Device> device;
    std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();

    std::cout << "Listing available devices..." << std::endl;
    device = std::make_shared<dai::Device>(pipeline);
    auto outputFeaturesLeftQueue = device->getOutputQueue("trackedFeaturesLeft", 8, false);
    auto outputFeaturesRightQueue = device->getOutputQueue("trackedFeaturesRight", 8, false);
    std::string tfPrefix = "oak";
    dai::rosBridge::TrackedFeaturesConverter leftConverter(tfPrefix + "_left_camera_optical_frame", true);

    dai::rosBridge::TrackedFeaturesConverter rightConverter(tfPrefix + "_right_camera_optical_frame", true);

    dai::rosBridge::BridgePublisher<depthai_ros_msgs::TrackedFeatures, dai::TrackedFeatures> featuresPubL(
        outputFeaturesLeftQueue,
        pnh,
        std::string("features_left"),
        std::bind(&dai::rosBridge::TrackedFeaturesConverter::toRosMsg, &leftConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "features_left");

    featuresPubL.addPublisherCallback();

    dai::rosBridge::BridgePublisher<depthai_ros_msgs::TrackedFeatures, dai::TrackedFeatures> featuresPubR(
        outputFeaturesRightQueue,
        pnh,
        std::string("features_right"),
        std::bind(&dai::rosBridge::TrackedFeaturesConverter::toRosMsg, &rightConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "features_right");

    featuresPubR.addPublisherCallback();
    std::cout << "Ready." << std::endl;
    ros::spin();

    return 0;
}
