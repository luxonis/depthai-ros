#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

#include "camera_info_manager/camera_info_manager.h"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_bridge/TrackletConverter.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "ros/ros.h"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/ObjectTracker.hpp"

#include "depthai_ros_msgs/SpatialDetectionArray.h"
#include "sensor_msgs/Imu.h"
#include "stereo_msgs/DisparityImage.h"
#include "sensor_msgs/Image.h"
#include "vision_msgs/Detection2DArray.h"


const std::vector<std::string> label_map = {
    "person",        "bicycle",      "car",           "motorbike",     "aeroplane",   "bus",         "train",       "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",     "parking meter", "bench",       "bird",        "cat",         "dog",          "horse",
    "sheep",         "cow",          "elephant",      "bear",          "zebra",       "giraffe",     "backpack",    "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",       "skis",          "snowboard",   "sports ball", "kite",        "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket", "bottle",        "wine glass",  "cup",         "fork",        "knife",        "spoon",
    "bowl",          "banana",       "apple",         "sandwich",      "orange",      "broccoli",    "carrot",      "hot dog",      "pizza",
    "donut",         "cake",         "chair",         "sofa",          "pottedplant", "bed",         "diningtable", "toilet",       "tvmonitor",
    "laptop",        "mouse",        "remote",        "keyboard",      "cell phone",  "microwave",   "oven",        "toaster",      "sink",
    "refrigerator",  "book",         "clock",         "vase",          "scissors",    "teddy bear",  "hair drier",  "toothbrush"};

std::tuple<dai::Pipeline, int, int, int, int> createPipeline(bool syncNN,
                                                             bool subpixel,
                                                             bool lrcheck,
                                                             bool extended,
                                                             std::string nnPath,
                                                             int confidence,
                                                             int LRchecktresh,
                                                             int mono_fps,
                                                             int rgb_fps,
                                                             int previewWidth,
                                                             int previewHeight,
                                                             int rgbScaleNumerator,
                                                             int rgbScaleDinominator,
                                                             std::string monoResolutionStr,
                                                             std::string rgbResolutionStr) {
    dai::Pipeline pipeline;
    pipeline.setXLinkChunkSize(0);

    auto controlIn = pipeline.create<dai::node::XLinkIn>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto spatialDetectionNetwork = pipeline.create<dai::node::YoloSpatialDetectionNetwork>();
    auto objectTracker = pipeline.create<dai::node::ObjectTracker>();
    auto imu = pipeline.create<dai::node::IMU>();

    // create xlink connections
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto xoutNN = pipeline.create<dai::node::XLinkOut>();
    auto xoutPreview = pipeline.create<dai::node::XLinkOut>();
    auto xoutTracker = pipeline.create<dai::node::XLinkOut>();
    auto xoutImu = pipeline.create<dai::node::XLinkOut>();

    controlIn->setStreamName("control");
    xoutPreview->setStreamName("preview");
    xoutRgb->setStreamName("rgb");
    xoutDepth->setStreamName("depth");
    xoutNN->setStreamName("detections");
    xoutTracker->setStreamName("tracklets");
    xoutImu->setStreamName("imu");

    // MonoCameras
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    int monoWidth, monoHeight;
    if(monoResolutionStr == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
        monoWidth = 1280;
        monoHeight = 720;
    } else if(monoResolutionStr == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
        monoWidth = 640;
        monoHeight = 400;
    } else if(monoResolutionStr == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
        monoWidth = 1280;
        monoHeight = 800;
    } else if(monoResolutionStr == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
        monoWidth = 640;
        monoHeight = 480;
    } else {
        ROS_ERROR("Invalid parameter. -> monoResolution: %s", monoResolutionStr.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }
    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoLeft->setFps(mono_fps);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoRight->setFps(mono_fps);

    // StereoDepth
    stereo->initialConfig.setConfidenceThreshold(confidence);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setSubpixel(subpixel);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setDepthAlign(dai::CameraBoardSocket::RGB);

    // ObjectTracker
    objectTracker->setTrackerType(dai::TrackerType::ZERO_TERM_COLOR_HISTOGRAM);
    objectTracker->setTrackerIdAssignmentPolicy(dai::TrackerIdAssignmentPolicy::UNIQUE_ID);

    // IMU
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(5);
    imu->setMaxBatchReports(20);  // Get one message only for now.

    // ColorCamera
    int rgbWidth, rgbHeight;
    dai::node::ColorCamera::Properties::SensorResolution rgbResolution;
    if(rgbResolutionStr == "1080p") {
        rgbResolution = dai::node::ColorCamera::Properties::SensorResolution::THE_1080_P;
        rgbWidth = 1920;
        rgbHeight = 1080;
    } else if(rgbResolutionStr == "4K") {
        rgbResolution = dai::node::ColorCamera::Properties::SensorResolution::THE_4_K;
        rgbWidth = 3840;
        rgbHeight = 2160;
    } else if(rgbResolutionStr == "12MP") {
        rgbResolution = dai::node::ColorCamera::Properties::SensorResolution::THE_12_MP;
        rgbWidth = 4056;
        rgbHeight = 3040;
    } else if(rgbResolutionStr == "13MP") {
        rgbResolution = dai::node::ColorCamera::Properties::SensorResolution::THE_13_MP;
        rgbWidth = 4208;
        rgbHeight = 3120;
    } else {
        ROS_ERROR("Invalid parameter. -> rgbResolution: %s", rgbResolutionStr.c_str());
        throw std::runtime_error("Invalid color camera resolution.");
    }
    colorCam->setBoardSocket(dai::CameraBoardSocket::RGB);
    colorCam->setPreviewSize(previewWidth, previewHeight);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->setFps(rgb_fps);
    colorCam->setIspScale(rgbScaleNumerator, rgbScaleDinominator);
    rgbWidth = rgbWidth * rgbScaleNumerator / rgbScaleDinominator;
    rgbHeight = rgbHeight * rgbScaleNumerator / rgbScaleDinominator;

    // SpatialDetections
    spatialDetectionNetwork->setBlobPath(nnPath);
    spatialDetectionNetwork->setConfidenceThreshold(0.5f);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);

    // yolo specific parameters
    spatialDetectionNetwork->setNumClasses(80);
    spatialDetectionNetwork->setCoordinateSize(4);
    spatialDetectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    spatialDetectionNetwork->setAnchorMasks({{"side13", {3, 4, 5}}, {"side26", {1, 2, 3}}});
    spatialDetectionNetwork->setIouThreshold(0.5f);

    // Link Control
    controlIn->out.link(monoRight->inputControl);
    controlIn->out.link(monoLeft->inputControl);

    // Link MonoCameras
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    // Link StereoCamera
    stereo->depth.link(spatialDetectionNetwork->inputDepth);

    // Link ColorCamera
    colorCam->preview.link(spatialDetectionNetwork->input);
    colorCam->isp.link(objectTracker->inputTrackerFrame);

    // Link SpatialDetectionNetwork
    spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);
    spatialDetectionNetwork->passthrough.link(objectTracker->inputDetectionFrame);
    spatialDetectionNetwork->passthrough.link(objectTracker->inputTrackerFrame);
    spatialDetectionNetwork->out.link(objectTracker->inputDetections);

    // Link ObjectTracker
    // objectTracker->setDetectionLabelsToTrack({47});
    objectTracker->passthroughDetectionFrame.link(xoutPreview->input);
    objectTracker->passthroughDetections.link(xoutNN->input);
    objectTracker->passthroughTrackerFrame.link(xoutRgb->input);
    objectTracker->out.link(xoutTracker->input);

    // Link IMU
    imu->out.link(xoutImu->input);

    return std::make_tuple(pipeline, rgbWidth, rgbHeight, monoWidth, monoHeight);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle pnh("~");

    std::string tfPrefix, resourceBaseFolder, nnPath;
    std::string camera_param_uri;
    std::string nnName(BLOB_NAME);  // Set your blob name for the model here
    bool lrcheck, extended, syncNN, subpixel, enableDotProjector, enableFloodLight;
    int rgbScaleNumerator, rgbScaleDinominator;
    double dotProjectormA, floodLightmA;
    double angularVelCovariance, linearAccelCovariance;
    int imuModeParam = 1;
    int badParams = 0;
    int confidence = 200;
    int LRchecktresh = 5;
    int monoFPS = 30;
    int rgbFPS = 30;
    int previewHeight = 416;
    int previewWidth = 416;
    std::string monoResolution = "720p", rgbResolution = "1080p";

    badParams += !pnh.getParam("tf_prefix", tfPrefix);
    badParams += !pnh.getParam("camera_param_uri", camera_param_uri);
    badParams += !pnh.getParam("sync_nn", syncNN);
    badParams += !pnh.getParam("subpixel", subpixel);
    badParams += !pnh.getParam("confidence", confidence);
    badParams += !pnh.getParam("LRchecktresh", LRchecktresh);
    badParams += !pnh.getParam("monoResolution", monoResolution);
    badParams += !pnh.getParam("resourceBaseFolder", resourceBaseFolder);
    badParams += !pnh.getParam("monoFPS", monoFPS);
    badParams += !pnh.getParam("rgbFPS", rgbFPS);
    badParams += !pnh.getParam("previewHeight", previewHeight);
    badParams += !pnh.getParam("previewWidth", previewWidth);
    badParams += !pnh.getParam("lrcheck", lrcheck);
    badParams += !pnh.getParam("extended", extended);
    badParams += !pnh.getParam("rgbScaleNumerator", rgbScaleNumerator);
    badParams += !pnh.getParam("rgbScaleDinominator", rgbScaleDinominator);
    badParams += !pnh.getParam("monoResolution", monoResolution);
    badParams += !pnh.getParam("rgbResolution", rgbResolution);

    badParams += !pnh.getParam("angularVelCovariance", angularVelCovariance);
    badParams += !pnh.getParam("linearAccelCovariance", linearAccelCovariance);

    badParams += !pnh.getParam("enableDotProjector", enableDotProjector);
    badParams += !pnh.getParam("enableFloodLight", enableFloodLight);
    badParams += !pnh.getParam("dotProjectormA", dotProjectormA);
    badParams += !pnh.getParam("floodLightmA", floodLightmA);

    if(badParams > 0) {
        throw std::runtime_error("Couldn't find one of the parameters");
    }

    std::string nnParam;
    pnh.getParam("nnName", nnParam);
    if(nnParam != "x") {
        pnh.getParam("nnName", nnName);
    }

    if(resourceBaseFolder.empty()) {
        throw std::runtime_error("Send the path to the resouce folder containing NNBlob in \'resourceBaseFolder\' ");
    }
    nnPath = resourceBaseFolder + "/" + nnName;

    dai::Pipeline pipeline;
    int rgbWidth, rgbHeight, monoWidth, monoHeight;
    std::tie(pipeline, rgbWidth, rgbHeight, monoWidth, monoHeight) = createPipeline(syncNN,
                                                                                    subpixel,
                                                                                    lrcheck,
                                                                                    extended,
                                                                                    nnPath,
                                                                                    confidence,
                                                                                    LRchecktresh,
                                                                                    monoFPS,
                                                                                    rgbFPS,
                                                                                    previewWidth,
                                                                                    previewHeight,
                                                                                    rgbScaleNumerator,
                                                                                    rgbScaleDinominator,
                                                                                    monoResolution,
                                                                                    rgbResolution);

    // Device
    std::shared_ptr<dai::Device> device;
    device = std::make_shared<dai::Device>(pipeline);

    // Get Input/Output Queues
    auto previewQueue = device->getOutputQueue("preview", 30, false);
    auto colorQueue = device->getOutputQueue("rgb", 30, false);
    auto detectionQueue = device->getOutputQueue("detections", 30, false);
    auto depthQueue = device->getOutputQueue("depth", 30, false);
    auto imuQueue = device->getOutputQueue("imu", 30, false);
    auto controlQueue = device->getInputQueue("control");
    auto trackletQueue = device->getOutputQueue("tracklets", 30, false);

    auto calibrationHandler = device->readCalibration();

    std::vector<std::tuple<std::string, int, int>> irDrivers = device->getIrDrivers();
    if(!irDrivers.empty()) {
        if(enableDotProjector) device->setIrLaserDotProjectorBrightness(dotProjectormA);  // in mA, 0..1200
        if(enableFloodLight) device->setIrFloodLightBrightness(floodLightmA);             // in mA, 0..1500
    }

    auto boardName = calibrationHandler.getEepromData().boardName;
    if(monoHeight > 480 && boardName == "OAK-D-LITE") {
        ROS_ERROR("Invalid parameter. -> monoResolution: %s", monoResolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution for OAK-D-LITE.");
    }

    // IMU
    dai::ros::ImuSyncMethod imuMode = static_cast<dai::ros::ImuSyncMethod>(imuModeParam);
    dai::rosBridge::ImuConverter imuConverter(tfPrefix + "_imu_frame", imuMode, linearAccelCovariance, angularVelCovariance);
    dai::rosBridge::BridgePublisher<sensor_msgs::Imu, dai::IMUData> imuPublish(
        imuQueue,
        pnh,
        std::string("imu"),
        std::bind(&dai::rosBridge::ImuConverter::toRosMsg, &imuConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "imu");
    imuPublish.addPublisherCallback();

    // RGB
    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, rgbWidth, rgbHeight);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(
        colorQueue,
        pnh,
        std::string("color/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        rgbCameraInfo,
        "color");
    rgbPublish.addPublisherCallback();

    // Depth
    dai::rosBridge::ImageConverter depthConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    auto depthCameraInfo = depthConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, monoWidth, monoHeight);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(
        depthQueue,
        pnh,
        std::string("stereo/depth"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &depthConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        depthCameraInfo,
        "stereo");
    depthPublish.addPublisherCallback();

    // Preview
    dai::rosBridge::ImageConverter previewConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    auto previewCameraInfo = previewConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, previewWidth, previewHeight);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> previewPublish(
        previewQueue,
        pnh,
        std::string("color/preview/image"),
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &previewConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        previewCameraInfo,
        "color/preview");
    previewPublish.addPublisherCallback();

    // SpatialDetections
    dai::rosBridge::SpatialDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", previewWidth, previewHeight, false);
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(
        detectionQueue,
        pnh,
        std::string("detections/yolov4_spatial"),
        std::bind(&dai::rosBridge::SpatialDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
        30);
    detectionPublish.addPublisherCallback();

    // Tracklets
    dai::rosBridge::TrackletConverter trackConverter(tfPrefix + "_rgb_camera_optical_frame", previewWidth, previewHeight, rgbWidth, rgbHeight, false, false);
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::TrackletArray, dai::Tracklets> trackletPublish(
        trackletQueue,
        pnh,
        std::string("tracker/tracklets"),
        std::bind(&dai::rosBridge::TrackletConverter::toRosMsg, &trackConverter, std::placeholders::_1, std::placeholders::_2),
        30);
    trackletPublish.addPublisherCallback();

    ros::spin();

    return 0;
}
