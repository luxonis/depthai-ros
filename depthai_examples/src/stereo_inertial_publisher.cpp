#include <cstdio>
#include "depthai/pipeline/InputQueue.hpp"
#include "depthai/pipeline/node/host/RGBD.hpp"
#include <functional>
#include <iostream>
#include <tuple>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai_bridge/PointCloudConverter.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

std::vector<std::string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};

struct OutputQueues {
    std::shared_ptr<dai::MessageQueue> stereoOut;
    std::shared_ptr<dai::MessageQueue> rgbOut;
    std::shared_ptr<dai::MessageQueue> leftOut;
    std::shared_ptr<dai::MessageQueue> rightOut;
    std::shared_ptr<dai::MessageQueue> imuOut;
    std::shared_ptr<dai::MessageQueue> previewOut;
    std::shared_ptr<dai::MessageQueue> detectionOut;
    std::shared_ptr<dai::MessageQueue> pclOut;
    std::shared_ptr<dai::InputQueue> controlLeft;
    std::shared_ptr<dai::InputQueue> controlRight;
    std::shared_ptr<dai::InputQueue> controlRgb;
};
struct PipelineOpts {
    std::string nnName;
    bool lrcheck;
    bool extended;
    bool subpixel;
    int stereoFPS;
    int rgbWidth;
    int rgbHeight;
    int monoWidth;
    int monoHeight;
    int confidence;
    int lrCheckThresh;
};
OutputQueues createPipeline(dai::Pipeline& pipeline, PipelineOpts opts) {
    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B, {}, opts.stereoFPS);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C, {}, opts.stereoFPS);
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto rgbd = pipeline.create<dai::node::RGBD>()->build();
    auto imu = pipeline.create<dai::node::IMU>();
    OutputQueues queues;
    rgbd->setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::METER);

    queues.controlLeft = monoLeft->inputControl.createInputQueue(8, false);
    queues.controlRight = monoRight->inputControl.createInputQueue(8, false);

    // StereoDepth
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->setLeftRightCheck(opts.lrcheck);
    stereo->setExtendedDisparity(opts.extended);
    stereo->setSubpixel(opts.subpixel);

    // Imu
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(5);
    imu->setMaxBatchReports(20);  // Get one message only for now.
    queues.imuOut = imu->out.createOutputQueue(8, false);

    // RGB image
    auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
    auto rgbOut = camRgb->requestOutput({opts.rgbWidth, opts.rgbHeight});
    queues.rgbOut = rgbOut->createOutputQueue(8, false);
    queues.controlRgb = camRgb->inputControl.createInputQueue(8, false);
    rgbOut->link(stereo->inputAlignTo);

    stereo->depth.link(rgbd->inDepth);
    auto* out = camRgb->requestOutput(std::pair<int, int>(640, 400), dai::ImgFrame::Type::RGB888i);
    out->link(rgbd->inColor);

    auto spatialDetectionNetwork = pipeline.create<dai::node::SpatialDetectionNetwork>();
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5f);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);
    dai::NNModelDescription modelDesc;
    modelDesc.model = opts.nnName;
    spatialDetectionNetwork->build(camRgb, stereo, modelDesc, 30);  // 30 FPS
    queues.detectionOut = spatialDetectionNetwork->out.createOutputQueue(8, false);
    queues.previewOut = spatialDetectionNetwork->passthrough.createOutputQueue(8, false);
    queues.pclOut = rgbd->pcl.createOutputQueue(8, false);
    stereo->setRectifyEdgeFillColor(0);
    auto monoOutLeft = monoLeft->requestOutput(std::make_pair(opts.monoWidth, opts.monoHeight));
    monoOutLeft->link(stereo->left);
    auto monoOutRight = monoRight->requestOutput(std::make_pair(opts.monoWidth, opts.monoHeight));
    monoOutRight->link(stereo->right);

    std::shared_ptr<dai::MessageQueue> stereoOut;
    stereoOut = stereo->depth.createOutputQueue(8, false);
    queues.stereoOut = stereoOut;

    return queues;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_inertial_node");

    std::string mxId = node->declare_parameter<std::string>("mxId", "");
    std::string tfPrefix = node->declare_parameter<std::string>("tfPrefix", "oak");
    std::string nnName = node->declare_parameter<std::string>("nnName", "yolov6-nano");
    int imuModeParam = node->declare_parameter<int>("imuMode", 1);
    bool lrcheck = node->declare_parameter<bool>("lrcheck", true);
    bool extended = node->declare_parameter<bool>("extended", false);
    bool subpixel = node->declare_parameter<bool>("subpixel", true);
    int rgbWidth = node->declare_parameter<int>("rgbWidth", 640);
    int rgbHeight = node->declare_parameter<int>("rgbHeight", 400);
    int monoWidth = node->declare_parameter<int>("monoWidth", 640);
    int monoHeight = node->declare_parameter<int>("monoHeight", 400);
    int stereoFPS = node->declare_parameter<int>("stereoFPS", 30);
    int confidence = node->declare_parameter<int>("confidence", 200);
    int lrCheckThresh = node->declare_parameter<int>("lrCheckThresh", 5);
    bool manualExposure = node->declare_parameter<bool>("manualExposure", false);
    int expTime = node->declare_parameter<int>("expTime", 20000);
    int sensIso = node->declare_parameter<int>("sensIso", 800);
    double angularVelCovariance = node->declare_parameter<double>("angularVelCovariance", 0.02);
    double linearAccelCovariance = node->declare_parameter<double>("linearAccelCovariance", 0.0);
    bool enableDotProjector = node->declare_parameter<bool>("enableDotProjector", false);
    bool enableFloodLight = node->declare_parameter<bool>("enableFloodLight", false);
    double dotProjectorIntensity = node->declare_parameter<double>("dotProjectorIntensity", 0.5);
    double floodLightIntensity = node->declare_parameter<double>("floodLightIntensity", 0.5);
    double enableRosBaseTimeUpdate = node->declare_parameter<bool>("enableRosBaseTimeUpdate", false);

    depthai_bridge::ImuSyncMethod imuMode = static_cast<depthai_bridge::ImuSyncMethod>(imuModeParam);
    std::shared_ptr<dai::Device> device;

    device = std::make_shared<dai::Device>();

    dai::Pipeline pipeline(device);
    PipelineOpts opts = {nnName, lrcheck, extended, subpixel, stereoFPS, rgbWidth, rgbHeight, monoWidth, monoHeight, confidence, lrCheckThresh};
    auto queues = createPipeline(pipeline, opts);

    // Set manual exposure
    if(manualExposure) {
        auto ctrl = std::make_shared<dai::CameraControl>();
        ctrl->setManualExposure(expTime, sensIso);
        queues.controlLeft->send(ctrl);
        queues.controlRight->send(ctrl);
        queues.controlRgb->send(ctrl);
    }

    pipeline.start();

    std::vector<std::tuple<std::string, int, int>> irDrivers = device->getIrDrivers();
    if(!irDrivers.empty()) {
        if(enableDotProjector) {
            device->setIrLaserDotProjectorIntensity(dotProjectorIntensity);
        }

        if(enableFloodLight) {
            device->setIrFloodLightIntensity(floodLightIntensity);
        }
    }

    depthai_bridge::ImageConverter rightConverter(tfPrefix + "_right_camera_optical_frame", true);
    if(enableRosBaseTimeUpdate) {
        rightConverter.setUpdateRosBaseTimeOnToRosMsg();
    }

    depthai_bridge::ImuConverter imuConverter(tfPrefix + "_imu_frame", imuMode, linearAccelCovariance, angularVelCovariance);
    if(enableRosBaseTimeUpdate) {
        imuConverter.setUpdateRosBaseTimeOnToRosMsg();
    }
    depthai_bridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData> imuPublish(
        queues.imuOut,
        node,
        "imu",
        std::bind(&depthai_bridge::ImuConverter::toRosMsg, &imuConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "imu");

    imuPublish.addPublisherCallback();

    depthai_bridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    if(enableRosBaseTimeUpdate) {
        rgbConverter.setUpdateRosBaseTimeOnToRosMsg();
    }
    auto calibrationHandler = device->readCalibration();
    auto pub = std::make_unique<depthai_bridge::TFPublisher>(node,
                                    calibrationHandler,
                                    device->getConnectedCameraFeatures(),
                                    "oak",
                                    device->getDeviceName(),
                                    "oak",
                                    "parent_frame",
                                    "0.0",
                                    "0,0",
                                    "0.0",
                                    "0.0",
                                    "0.0",
                                    "0.0",
                                    "false",
                                    "",
                                    "",
                                    false);
    while(pipeline.isRunning() && rclcpp::ok()) {
        depthai_bridge::PointCloudConverter pclConv(tfPrefix + "_rgb_camera_optical_frame", false);
        depthai_bridge::BridgePublisher<sensor_msgs::msg::PointCloud2, dai::PointCloudData> pclPublish(
            queues.pclOut,
            node,
            "stereo/points",
            std::bind(&depthai_bridge::PointCloudConverter::toRosMsg,
                      pclConv,
                      std::placeholders::_1,
                      std::placeholders::_2),
            30,
            "",
            "pcl"
        );
        pclPublish.addPublisherCallback();
        auto rightCameraInfo = rightConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);
        auto depthCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, rgbWidth, rgbHeight);
        auto depthconverter = rgbConverter;

        depthai_bridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(
            queues.stereoOut,
            node,
            "stereo/depth",
            std::bind(&depthai_bridge::ImageConverter::toRosMsg,
                      &depthconverter,  // since the converter has the same frame name
                                        // and image type is also same we can reuse it
                      std::placeholders::_1,
                      std::placeholders::_2),
            30,
            depthCameraInfo,
            "stereo");
        depthPublish.addPublisherCallback();

        auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, rgbWidth, rgbHeight);
        auto imgQueue = queues.rgbOut;
        depthai_bridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(
            imgQueue,
            node,
            "color/image",
            std::bind(&depthai_bridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
            30,
            rgbCameraInfo,
            "color");
        rgbPublish.addPublisherCallback();

        auto previewQueue = queues.previewOut;
        auto detectionQueue = queues.detectionOut;
        auto previewCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A);

        depthai_bridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> previewPublish(
            previewQueue,
            node,
            "color/preview/image",
            std::bind(&depthai_bridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
            30,
            previewCameraInfo,
            "color/preview");
        previewPublish.addPublisherCallback();

        depthai_bridge::SpatialDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", false);
        depthai_bridge::BridgePublisher<depthai_ros_msgs::msg::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(
            detectionQueue,
            node,
            "color/yolov4_Spatial_detections",
            std::bind(&depthai_bridge::SpatialDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
            30);
        detectionPublish.addPublisherCallback();
        rclcpp::spin(node);
    }
    return 0;
}
