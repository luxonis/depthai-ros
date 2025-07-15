#include <cstdio>
#include <depthai/pipeline/InputQueue.hpp>
#include <functional>
#include <iostream>
#include <tuple>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"

std::vector<std::string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};

struct OutputQueues {
    std::shared_ptr<dai::MessageQueue> stereoOut;
    std::shared_ptr<dai::MessageQueue> rgbOut;
    std::shared_ptr<dai::MessageQueue> leftOut;
    std::shared_ptr<dai::MessageQueue> rightOut;
    std::shared_ptr<dai::MessageQueue> imuOut;
    std::shared_ptr<dai::MessageQueue> previewOut;
    std::shared_ptr<dai::MessageQueue> detectionOut;
    std::shared_ptr<dai::InputQueue> controlLeft;
    std::shared_ptr<dai::InputQueue> controlRight;
    std::shared_ptr<dai::InputQueue> controlRgb;
};
struct PipelineOpts {
    bool enableDepth;
    bool enableSpatialDetection;
    bool lrcheck;
    bool extended;
    bool subpixel;
    bool rectify;
    bool depthAligned;
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
    auto imu = pipeline.create<dai::node::IMU>();
    OutputQueues queues;

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

    if(opts.depthAligned) {
        // RGB image
        auto camRgb = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);
        auto rgbOut = camRgb->requestOutput({opts.rgbWidth, opts.rgbHeight});
        queues.rgbOut = rgbOut->createOutputQueue(8, false);
        queues.controlRgb = camRgb->inputControl.createInputQueue(8, false);
        rgbOut->link(stereo->inputAlignTo);

        if(opts.enableSpatialDetection) {
            auto spatialDetectionNetwork = pipeline.create<dai::node::SpatialDetectionNetwork>();
            spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5f);
            spatialDetectionNetwork->setDepthLowerThreshold(100);
            spatialDetectionNetwork->setDepthUpperThreshold(5000);
            dai::NNModelDescription modelDesc;
            modelDesc.model = "yolov6-nano";
            spatialDetectionNetwork->build(camRgb, stereo, modelDesc, 30);  // 30 FPS
            queues.detectionOut = spatialDetectionNetwork->out.createOutputQueue(8, false);
            queues.previewOut = spatialDetectionNetwork->passthrough.createOutputQueue(8, false);
        }
    }

    stereo->setRectifyEdgeFillColor(0);
    auto monoOutLeft = monoLeft->requestOutput(std::make_pair(opts.monoWidth, opts.monoHeight));
    monoOutLeft->link(stereo->left);
    auto monoOutRight = monoRight->requestOutput(std::make_pair(opts.monoWidth, opts.monoHeight));
    monoOutRight->link(stereo->right);
    if(opts.rectify) {
        queues.leftOut = stereo->rectifiedLeft.createOutputQueue(8, false);
        queues.rightOut = stereo->rectifiedRight.createOutputQueue(8, false);
    } else {
        queues.leftOut = monoOutLeft->createOutputQueue(8, false);
        queues.rightOut = monoOutRight->createOutputQueue(8, false);
    }

    std::shared_ptr<dai::MessageQueue> stereoOut;
    if(opts.enableDepth) {
        stereoOut = stereo->depth.createOutputQueue(8, false);
    } else {
        stereoOut = stereo->disparity.createOutputQueue(8, false);
    }
    queues.stereoOut = stereoOut;

    return queues;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_inertial_node");

    std::string tfPrefix, mode, mxId, resourceBaseFolder, nnPath;
    int stereo_fps, confidence, LRchecktresh, imuModeParam, detectionClassesCount, expTime, sensIso;
    int rgbWidth, rgbHeight, monoWidth, monoHeight;
    bool lrcheck, extended, subpixel, enableDepth, rectify, depth_aligned, manualExposure;
    bool enableSpatialDetection, enableDotProjector, enableFloodLight;
    bool usb2Mode, poeMode, syncNN;
    double angularVelCovariance, linearAccelCovariance;
    double dotProjectorIntensity, floodLightIntensity;
    bool enableRosBaseTimeUpdate;

    node->declare_parameter("mxId", "");
    node->declare_parameter("usb2Mode", false);
    node->declare_parameter("poeMode", false);
    node->declare_parameter("resourceBaseFolder", "");

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("mode", "depth");
    node->declare_parameter("imuMode", 1);

    node->declare_parameter("lrcheck", true);
    node->declare_parameter("extended", false);
    node->declare_parameter("subpixel", true);
    node->declare_parameter("rectify", false);

    node->declare_parameter("depth_aligned", true);
    node->declare_parameter("rgb_width", 1280);
    node->declare_parameter("rgb_height", 720);
    node->declare_parameter("mono_width", 640);
    node->declare_parameter("mono_height", 400);
    node->declare_parameter("stereo_fps", 30);
    node->declare_parameter("confidence", 200);
    node->declare_parameter("LRchecktresh", 5);
    node->declare_parameter("manualExposure", false);
    node->declare_parameter("expTime", 20000);
    node->declare_parameter("sensIso", 800);

    node->declare_parameter("previewWidth", 416);
    node->declare_parameter("previewHeight", 416);

    node->declare_parameter("angularVelCovariance", 0.02);
    node->declare_parameter("linearAccelCovariance", 0.0);
    node->declare_parameter("enableSpatialDetection", true);
    node->declare_parameter("detectionClassesCount", 80);
    node->declare_parameter("syncNN", true);
    node->declare_parameter("nnName", "x");

    node->declare_parameter("enableDotProjector", false);
    node->declare_parameter("enableFloodLight", false);
    node->declare_parameter("dotProjectorIntensity", 0.5);
    node->declare_parameter("floodLightIntensity", 0.5);
    node->declare_parameter("enableRosBaseTimeUpdate", false);

    // updating parameters if defined in launch file.

    node->get_parameter("mxId", mxId);
    node->get_parameter("usb2Mode", usb2Mode);
    node->get_parameter("poeMode", poeMode);
    node->get_parameter("resourceBaseFolder", resourceBaseFolder);

    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("mode", mode);
    node->get_parameter("imuMode", imuModeParam);

    node->get_parameter("lrcheck", lrcheck);
    node->get_parameter("extended", extended);
    node->get_parameter("subpixel", subpixel);
    node->get_parameter("rectify", rectify);

    node->get_parameter("depth_aligned", depth_aligned);
    node->get_parameter("stereo_fps", stereo_fps);
    node->get_parameter("confidence", confidence);
    node->get_parameter("LRchecktresh", LRchecktresh);
    node->get_parameter("manualExposure", manualExposure);
    node->get_parameter("expTime", expTime);
    node->get_parameter("sensIso", sensIso);

    node->get_parameter("rgb_width", rgbWidth);
    node->get_parameter("rgb_height", rgbHeight);
    node->get_parameter("mono_width", monoWidth);
    node->get_parameter("mono_height", monoHeight);

    node->get_parameter("angularVelCovariance", angularVelCovariance);
    node->get_parameter("linearAccelCovariance", linearAccelCovariance);
    node->get_parameter("enableSpatialDetection", enableSpatialDetection);
    node->get_parameter("detectionClassesCount", detectionClassesCount);
    node->get_parameter("syncNN", syncNN);

    node->get_parameter("enableDotProjector", enableDotProjector);
    node->get_parameter("enableFloodLight", enableFloodLight);
    node->get_parameter("dotProjectorIntensity", dotProjectorIntensity);
    node->get_parameter("floodLightIntensity", floodLightIntensity);
    node->get_parameter("enableRosBaseTimeUpdate", enableRosBaseTimeUpdate);

    if(mode == "depth") {
        enableDepth = true;
    } else {
        enableDepth = false;
    }

    dai::ros::ImuSyncMethod imuMode = static_cast<dai::ros::ImuSyncMethod>(imuModeParam);
    std::shared_ptr<dai::Device> device;
    // std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();

    device = std::make_shared<dai::Device>();

    dai::Pipeline pipeline(device);
    bool isDeviceFound = false;
    PipelineOpts opts = {
        enableDepth,
        enableSpatialDetection,
        lrcheck,
        extended,
        subpixel,
        rectify,
        depth_aligned,
        stereo_fps,
        rgbWidth,
        rgbHeight,
        monoWidth,
        monoHeight,
        confidence,
        LRchecktresh
    };
    auto queues = createPipeline(pipeline,opts);

    // std::cout << "Listing available devices..." << std::endl;
    // for(auto deviceInfo : availableDevices) {
    //     std::cout << "Device Mx ID: " << deviceInfo.getDeviceId() << std::endl;
    //     if(deviceInfo.getDeviceId() == mxId) {
    //         if(deviceInfo.state == X_LINK_UNBOOTED || deviceInfo.state == X_LINK_BOOTLOADER) {
    //             isDeviceFound = true;
    //             if(poeMode) {
    //                 device = std::make_shared<dai::Device>();
    //             } else {
    //                 device = std::make_shared<dai::Device>();
    //             }
    //             break;
    //         } else if(deviceInfo.state == X_LINK_BOOTED) {
    //             throw std::runtime_error("\" DepthAI Device with MxId  \"" + mxId + "\" is already booted on different process.  \"");
    //         }
    //     } else if(mxId == "x") {
    //         isDeviceFound = true;
    //         device = std::make_shared<dai::Device>();
    //     }
    // }
    // if(!isDeviceFound) {
    //     throw std::runtime_error("\" DepthAI Device with MxId  \"" + mxId + "\" not found.  \"");
    // }

    if(!poeMode) {
        std::cout << "Device USB status: " << usbStrings[static_cast<int32_t>(device->getUsbSpeed())] << std::endl;
    }


    // Set manual exposure
    if(manualExposure) {
        auto ctrl = std::make_shared<dai::CameraControl>();
        ctrl->setManualExposure(expTime, sensIso);
        queues.controlLeft->send(ctrl);
        queues.controlRight->send(ctrl);
        if(depth_aligned) {
            queues.controlRgb->send(ctrl);
        }
    }


    pipeline.start();
    auto calibrationHandler = device->readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    std::vector<std::tuple<std::string, int, int>> irDrivers = device->getIrDrivers();
    if(!irDrivers.empty()) {
        if(enableDotProjector) {
            device->setIrLaserDotProjectorIntensity(dotProjectorIntensity);
        }

        if(enableFloodLight) {
            device->setIrFloodLightIntensity(floodLightIntensity);
        }
    }

    dai::rosBridge::ImageConverter converter(tfPrefix + "_left_camera_optical_frame", true);
    if(enableRosBaseTimeUpdate) {
        converter.setUpdateRosBaseTimeOnToRosMsg();
    }
    dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
    if(enableRosBaseTimeUpdate) {
        rightconverter.setUpdateRosBaseTimeOnToRosMsg();
    }
    const std::string leftPubName = rectify ? std::string("left/image_rect") : std::string("left/image_raw");
    const std::string rightPubName = rectify ? std::string("right/image_rect") : std::string("right/image_raw");

    dai::rosBridge::ImuConverter imuConverter(tfPrefix + "_imu_frame", imuMode, linearAccelCovariance, angularVelCovariance);
    if(enableRosBaseTimeUpdate) {
        imuConverter.setUpdateRosBaseTimeOnToRosMsg();
    }
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData> imuPublish(
        queues.imuOut,
        node,
        std::string("imu"),
        std::bind(&dai::rosBridge::ImuConverter::toRosMsg, &imuConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "imu");

    imuPublish.addPublisherCallback();

    // auto leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, monoWidth, monoHeight);
    // auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);
    // const std::string leftPubName = rectify ? std::string("left/image_rect") : std::string("left/image_raw");
    // const std::string rightPubName = rectify ? std::string("right/image_rect") : std::string("right/image_raw");

    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    if(enableRosBaseTimeUpdate) {
        rgbConverter.setUpdateRosBaseTimeOnToRosMsg();
    }
    while(pipeline.isRunning()){
    if(enableDepth) {
        auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, rgbWidth, rgbHeight);
        auto depthCameraInfo =
            depth_aligned ? rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, rgbWidth, rgbHeight) : rightCameraInfo;
        auto depthconverter = depth_aligned ? rgbConverter : rightconverter;

        dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(
            queues.stereoOut,
            node,
            std::string("stereo/depth"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                      &depthconverter,  // since the converter has the same frame name
                                        // and image type is also same we can reuse it
                      std::placeholders::_1,
                      std::placeholders::_2),
            30,
            depthCameraInfo,
            "stereo");
        depthPublish.addPublisherCallback();

        if(depth_aligned) {
            auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, rgbWidth, rgbHeight);
            auto imgQueue = queues.rgbOut;
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(
                imgQueue,
                node,
                std::string("color/image"),
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
                30,
                rgbCameraInfo,
                "color");
            rgbPublish.addPublisherCallback();

            if(enableSpatialDetection) {
                auto previewQueue = queues.previewOut;
                auto detectionQueue = queues.detectionOut;
                auto previewCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A);

                dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> previewPublish(
                    previewQueue,
                    node,
                    std::string("color/preview/image"),
                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
                    30,
                    previewCameraInfo,
                    "color/preview");
                previewPublish.addPublisherCallback();

                dai::rosBridge::SpatialDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", 416, 416, false);
                dai::rosBridge::BridgePublisher<depthai_ros_msgs::msg::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(
                    detectionQueue,
                    node,
                    std::string("color/yolov4_Spatial_detections"),
                    std::bind(&dai::rosBridge::SpatialDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
                    30);
                detectionPublish.addPublisherCallback();
                rclcpp::spin(node);
            }
            rclcpp::spin(node);
        } else {
            auto leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, monoWidth, monoHeight);
            auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);

            auto leftQueue = queues.leftOut;
            auto rightQueue = queues.rightOut;
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftPublish(
                leftQueue,
                node,
                leftPubName,
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &converter, std::placeholders::_1, std::placeholders::_2),
                30,
                leftCameraInfo,
                "left");
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(
                rightQueue,
                node,
                rightPubName,
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rightconverter, std::placeholders::_1, std::placeholders::_2),
                30,
                rightCameraInfo,
                "right");
            rightPublish.addPublisherCallback();
            leftPublish.addPublisherCallback();
            rclcpp::spin(node);
        }
    } else {
        std::string tfSuffix = depth_aligned ? "_rgb_camera_optical_frame" : "_right_camera_optical_frame";
        dai::rosBridge::DisparityConverter dispConverter(tfPrefix + tfSuffix, 880, 7.5, 20, 2000);  // TODO(sachin): undo hardcoding of baseline
        auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoHeight, monoWidth);

        auto disparityCameraInfo =
            depth_aligned ? rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, 1280, 720) : rightCameraInfo;
        auto depthconverter = depth_aligned ? rgbConverter : rightconverter;
        dai::rosBridge::BridgePublisher<stereo_msgs::msg::DisparityImage, dai::ImgFrame> dispPublish(
            queues.stereoOut,
            node,
            std::string("stereo/disparity"),
            std::bind(&dai::rosBridge::DisparityConverter::toRosMsg, &dispConverter, std::placeholders::_1, std::placeholders::_2),
            30,
            disparityCameraInfo,
            "stereo");
        dispPublish.addPublisherCallback();

        if(depth_aligned) {
            auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, rgbWidth, rgbHeight);
            auto imgQueue = queues.rgbOut;
            dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(
                imgQueue,
                node,
                std::string("color/image"),
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
                30,
                rgbCameraInfo,
                "color");
            rgbPublish.addPublisherCallback();
            if(enableSpatialDetection) {
                auto previewQueue = queues.previewOut;
                auto detectionQueue = queues.detectionOut;
                auto previewCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A);

                dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> previewPublish(
                    previewQueue,
                    node,
                    std::string("color/preview/image"),
                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
                    30,
                    previewCameraInfo,
                    "color/preview");
                previewPublish.addPublisherCallback();

                dai::rosBridge::SpatialDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", 416, 416, false);
                dai::rosBridge::BridgePublisher<depthai_ros_msgs::msg::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(
                    detectionQueue,
                    node,
                    std::string("color/yolov4_Spatial_detections"),
                    std::bind(&dai::rosBridge::SpatialDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
                    30);
                detectionPublish.addPublisherCallback();
                rclcpp::spin(node);
            }
            rclcpp::spin(node);
        } else {
            auto leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, monoWidth, monoHeight);
            auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);

            auto leftQueue = queues.leftOut;
            auto rightQueue = queues.rightOut;
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftPublish(
                leftQueue,
                node,
                leftPubName,
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &converter, std::placeholders::_1, std::placeholders::_2),
                30,
                leftCameraInfo,
                "left");
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(
                rightQueue,
                node,
                rightPubName,
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rightconverter, std::placeholders::_1, std::placeholders::_2),
                30,
                rightCameraInfo,
                "right");
            rightPublish.addPublisherCallback();
            leftPublish.addPublisherCallback();
            rclcpp::spin(node);
        }
    }

    }
    return 0;
}
