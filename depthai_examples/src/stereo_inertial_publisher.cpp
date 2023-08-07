
#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

#include "camera_info_manager/camera_info_manager.h"
#include "depthai_ros_msgs/SpatialDetectionArray.h"
#include "ros/node_handle.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "stereo_msgs/DisparityImage.h"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"

std::vector<std::string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};

std::tuple<dai::Pipeline, int, int> createPipeline(bool enableDepth,
                                                   bool enableSpatialDetection,
                                                   bool lrcheck,
                                                   bool extended,
                                                   bool subpixel,
                                                   bool rectify,
                                                   bool depth_aligned,
                                                   int stereo_fps,
                                                   int confidence,
                                                   int LRchecktresh,
                                                   int detectionClassesCount,
                                                   std::string stereoResolution,
                                                   std::string rgbResolutionStr,
                                                   int rgbScaleNumerator,
                                                   int rgbScaleDinominator,
                                                   int previewWidth,
                                                   int previewHeight,
                                                   bool syncNN,
                                                   std::string nnPath) {
    dai::Pipeline pipeline;
    pipeline.setXLinkChunkSize(0);
    auto controlIn = pipeline.create<dai::node::XLinkIn>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto imu = pipeline.create<dai::node::IMU>();
    auto xoutImu = pipeline.create<dai::node::XLinkOut>();

    controlIn->setStreamName("control");
    controlIn->out.link(monoRight->inputControl);
    controlIn->out.link(monoLeft->inputControl);

    if(enableDepth) {
        xoutDepth->setStreamName("depth");
    } else {
        xoutDepth->setStreamName("disparity");
    }

    xoutImu->setStreamName("imu");

    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    int stereoWidth, stereoHeight, rgbWidth, rgbHeight;
    if(stereoResolution == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
        stereoWidth = 1280;
        stereoHeight = 720;
    } else if(stereoResolution == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
        stereoWidth = 640;
        stereoHeight = 400;
    } else if(stereoResolution == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
        stereoWidth = 1280;
        stereoHeight = 800;
    } else if(stereoResolution == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
        stereoWidth = 640;
        stereoHeight = 480;
    } else {
        ROS_ERROR("Invalid parameter. -> monoResolution: %s", stereoResolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    // MonoCamera
    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoLeft->setFps(stereo_fps);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);
    monoRight->setFps(stereo_fps);

    // StereoDepth
    stereo->initialConfig.setConfidenceThreshold(confidence);        // Known to be best
    stereo->setRectifyEdgeFillColor(0);                              // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);  // Known to be best
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);
    if(enableDepth && depth_aligned) stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);

    // Imu
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(5);
    imu->setMaxBatchReports(20);  // Get one message only for now.

    if(depth_aligned) {
        // RGB image
        auto camRgb = pipeline.create<dai::node::ColorCamera>();
        auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
        xoutRgb->setStreamName("rgb");
        camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
        camRgb->setFps(stereo_fps);
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

        camRgb->setResolution(rgbResolution);

        rgbWidth = rgbWidth * rgbScaleNumerator / rgbScaleDinominator;
        rgbHeight = rgbHeight * rgbScaleNumerator / rgbScaleDinominator;
        camRgb->setIspScale(rgbScaleNumerator, rgbScaleDinominator);

        camRgb->isp.link(xoutRgb->input);

        // std::cout << (rgbWidth % 2 == 0 && rgbHeight % 3 == 0) << std::endl;
        // assert(("Needs Width to be multiple of 2 and height to be multiple of 3 since the Image is NV12 format here.", (rgbWidth % 2 == 0 && rgbHeight % 3 ==
        // 0)));
        if(rgbWidth % 16 != 0) {
            if(rgbResolution == dai::node::ColorCamera::Properties::SensorResolution::THE_12_MP) {
                ROS_ERROR_STREAM("RGB Camera width should be multiple of 16. Please choose a different scaling factor."
                                 << std::endl
                                 << "Here are the scalng options that works for 12MP with depth aligned" << std::endl
                                 << "4056 x 3040 *  2/13 -->  624 x  468" << std::endl
                                 << "4056 x 3040 *  2/39 -->  208 x  156" << std::endl
                                 << "4056 x 3040 *  2/51 -->  160 x  120" << std::endl
                                 << "4056 x 3040 *  4/13 --> 1248 x  936" << std::endl
                                 << "4056 x 3040 *  4/26 -->  624 x  468" << std::endl
                                 << "4056 x 3040 *  4/29 -->  560 x  420" << std::endl
                                 << "4056 x 3040 *  4/35 -->  464 x  348" << std::endl
                                 << "4056 x 3040 *  4/39 -->  416 x  312" << std::endl
                                 << "4056 x 3040 *  6/13 --> 1872 x 1404" << std::endl
                                 << "4056 x 3040 *  6/39 -->  624 x  468" << std::endl
                                 << "4056 x 3040 *  7/25 --> 1136 x  852" << std::endl
                                 << "4056 x 3040 *  8/26 --> 1248 x  936" << std::endl
                                 << "4056 x 3040 *  8/39 -->  832 x  624" << std::endl
                                 << "4056 x 3040 *  8/52 -->  624 x  468" << std::endl
                                 << "4056 x 3040 *  8/58 -->  560 x  420" << std::endl
                                 << "4056 x 3040 * 10/39 --> 1040 x  780" << std::endl
                                 << "4056 x 3040 * 10/59 -->  688 x  516" << std::endl
                                 << "4056 x 3040 * 12/17 --> 2864 x 2146" << std::endl
                                 << "4056 x 3040 * 12/26 --> 1872 x 1404" << std::endl
                                 << "4056 x 3040 * 12/39 --> 1248 x  936" << std::endl
                                 << "4056 x 3040 * 13/16 --> 3296 x 2470" << std::endl
                                 << "4056 x 3040 * 14/39 --> 1456 x 1092" << std::endl
                                 << "4056 x 3040 * 14/50 --> 1136 x  852" << std::endl
                                 << "4056 x 3040 * 14/53 --> 1072 x  804" << std::endl
                                 << "4056 x 3040 * 16/39 --> 1664 x 1248" << std::endl
                                 << "4056 x 3040 * 16/52 --> 1248 x  936" << std::endl);

            } else {
                ROS_ERROR_STREAM("RGB Camera width should be multiple of 16. Please choose a different scaling factor.");
            }
            throw std::runtime_error("Adjust RGB Camaera scaling.");
        }

        if(rgbWidth > stereoWidth || rgbHeight > stereoHeight) {
            ROS_WARN_STREAM(
                "RGB Camera resolution is heigher than the configured stereo resolution. Upscaling the stereo depth/disparity to match RGB camera resolution.");
        } else if(rgbWidth > stereoWidth || rgbHeight > stereoHeight) {
            ROS_WARN_STREAM(
                "RGB Camera resolution is heigher than the configured stereo resolution. Downscaling the stereo depth/disparity to match RGB camera "
                "resolution.");
        }

        if(enableSpatialDetection) {
            if(previewWidth > rgbWidth or previewHeight > rgbHeight) {
                ROS_ERROR_STREAM(
                    "Preview Image size should be smaller than the scaled resolution. Please adjust the scale parameters or the preview size accordingly.");
                throw std::runtime_error("Invalid Image Size");
            }

            camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
            camRgb->setInterleaved(false);
            camRgb->setPreviewSize(previewWidth, previewHeight);

            auto spatialDetectionNetwork = pipeline.create<dai::node::YoloSpatialDetectionNetwork>();
            auto xoutNN = pipeline.create<dai::node::XLinkOut>();
            auto xoutPreview = pipeline.create<dai::node::XLinkOut>();
            xoutPreview->setStreamName("preview");
            xoutNN->setStreamName("detections");

            spatialDetectionNetwork->setBlobPath(nnPath);
            spatialDetectionNetwork->setConfidenceThreshold(0.5f);
            spatialDetectionNetwork->input.setBlocking(false);
            spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
            spatialDetectionNetwork->setDepthLowerThreshold(100);
            spatialDetectionNetwork->setDepthUpperThreshold(10000);

            // yolo specific parameters
            spatialDetectionNetwork->setNumClasses(detectionClassesCount);
            spatialDetectionNetwork->setCoordinateSize(4);
            spatialDetectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
            spatialDetectionNetwork->setAnchorMasks({{"side13", {3, 4, 5}}, {"side26", {1, 2, 3}}});
            spatialDetectionNetwork->setIouThreshold(0.5f);

            // Link plugins CAM -> NN -> XLINK
            camRgb->preview.link(spatialDetectionNetwork->input);
            if(syncNN)
                spatialDetectionNetwork->passthrough.link(xoutPreview->input);
            else
                camRgb->preview.link(xoutPreview->input);
            spatialDetectionNetwork->out.link(xoutNN->input);
            stereo->depth.link(spatialDetectionNetwork->inputDepth);
        }

        stereoWidth = rgbWidth;
        stereoHeight = rgbHeight;
    } else {
        // Stereo imges
        auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
        auto xoutRight = pipeline.create<dai::node::XLinkOut>();
        // XLinkOut
        xoutLeft->setStreamName("left");
        xoutRight->setStreamName("right");
        if(rectify) {
            stereo->rectifiedLeft.link(xoutLeft->input);
            stereo->rectifiedRight.link(xoutRight->input);
        } else {
            stereo->syncedLeft.link(xoutLeft->input);
            stereo->syncedRight.link(xoutRight->input);
        }
    }

    // Link plugins CAM -> STEREO -> XLINK
    stereo->setRectifyEdgeFillColor(0);
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    if(enableDepth) {
        stereo->depth.link(xoutDepth->input);
    } else {
        stereo->disparity.link(xoutDepth->input);
    }

    imu->out.link(xoutImu->input);
    std::cout << stereoWidth << " " << stereoHeight << " " << rgbWidth << " " << rgbHeight << std::endl;
    return std::make_tuple(pipeline, stereoWidth, stereoHeight);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "stereo_inertial_node");
    ros::NodeHandle pnh("~");

    std::string tfPrefix, mode, mxId, resourceBaseFolder, nnPath;
    std::string monoResolution = "720p", rgbResolution = "1080p";
    int badParams = 0, stereo_fps, confidence, LRchecktresh, imuModeParam, detectionClassesCount, expTime, sensIso;
    int rgbScaleNumerator, rgbScaleDinominator, previewWidth, previewHeight;
    bool lrcheck, extended, subpixel, enableDepth, rectify, depth_aligned, manualExposure;
    bool enableSpatialDetection, enableDotProjector, enableFloodLight;
    bool enableRosBaseTimeUpdate;
    bool usb2Mode, poeMode, syncNN;
    double angularVelCovariance, linearAccelCovariance;
    double dotProjectormA, floodLightmA;
    std::string nnName(BLOB_NAME);  // Set your blob name for the model here

    badParams += !pnh.getParam("mxId", mxId);
    badParams += !pnh.getParam("usb2Mode", usb2Mode);
    badParams += !pnh.getParam("poeMode", poeMode);
    badParams += !pnh.getParam("resourceBaseFolder", resourceBaseFolder);

    badParams += !pnh.getParam("tf_prefix", tfPrefix);
    badParams += !pnh.getParam("mode", mode);
    badParams += !pnh.getParam("imuMode", imuModeParam);

    badParams += !pnh.getParam("lrcheck", lrcheck);
    badParams += !pnh.getParam("extended", extended);
    badParams += !pnh.getParam("subpixel", subpixel);
    badParams += !pnh.getParam("rectify", rectify);

    badParams += !pnh.getParam("depth_aligned", depth_aligned);
    badParams += !pnh.getParam("stereo_fps", stereo_fps);
    badParams += !pnh.getParam("confidence", confidence);
    badParams += !pnh.getParam("LRchecktresh", LRchecktresh);
    badParams += !pnh.getParam("monoResolution", monoResolution);
    badParams += !pnh.getParam("rgbResolution", rgbResolution);
    badParams += !pnh.getParam("manualExposure", manualExposure);
    badParams += !pnh.getParam("expTime", expTime);
    badParams += !pnh.getParam("sensIso", sensIso);

    badParams += !pnh.getParam("rgbScaleNumerator", rgbScaleNumerator);
    badParams += !pnh.getParam("rgbScaleDinominator", rgbScaleDinominator);
    badParams += !pnh.getParam("previewWidth", previewWidth);
    badParams += !pnh.getParam("previewHeight", previewHeight);

    badParams += !pnh.getParam("angularVelCovariance", angularVelCovariance);
    badParams += !pnh.getParam("linearAccelCovariance", linearAccelCovariance);
    badParams += !pnh.getParam("enableSpatialDetection", enableSpatialDetection);
    badParams += !pnh.getParam("detectionClassesCount", detectionClassesCount);
    badParams += !pnh.getParam("syncNN", syncNN);

    badParams += !pnh.getParam("enableRosBaseTimeUpdate", enableRosBaseTimeUpdate);

    // Applies only to PRO model
    badParams += !pnh.getParam("enableDotProjector", enableDotProjector);
    badParams += !pnh.getParam("enableFloodLight", enableFloodLight);
    badParams += !pnh.getParam("dotProjectormA", dotProjectormA);
    badParams += !pnh.getParam("floodLightmA", floodLightmA);

    if(badParams > 0) {
        std::cout << " Bad parameters -> " << badParams << std::endl;
        throw std::runtime_error("Couldn't find %d of the parameters");
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
    std::cout << " nnPath ,, " << nnPath << std::endl;
    if(mode == "depth") {
        enableDepth = true;
    } else {
        enableDepth = false;
    }

    dai::ros::ImuSyncMethod imuMode = static_cast<dai::ros::ImuSyncMethod>(imuModeParam);

    dai::Pipeline pipeline;
    int width, height;
    bool isDeviceFound = false;
    std::tie(pipeline, width, height) = createPipeline(enableDepth,
                                                       enableSpatialDetection,
                                                       lrcheck,
                                                       extended,
                                                       subpixel,
                                                       rectify,
                                                       depth_aligned,
                                                       stereo_fps,
                                                       confidence,
                                                       LRchecktresh,
                                                       detectionClassesCount,
                                                       monoResolution,
                                                       rgbResolution,
                                                       rgbScaleNumerator,
                                                       rgbScaleDinominator,
                                                       previewWidth,
                                                       previewHeight,
                                                       syncNN,
                                                       nnPath);

    std::shared_ptr<dai::Device> device;
    std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();

    std::cout << "Listing available devices..." << std::endl;
    for(auto deviceInfo : availableDevices) {
        std::cout << "Device Mx ID: " << deviceInfo.getMxId() << std::endl;
        if(deviceInfo.getMxId() == mxId) {
            if(deviceInfo.state == X_LINK_UNBOOTED || deviceInfo.state == X_LINK_BOOTLOADER) {
                isDeviceFound = true;
                if(poeMode) {
                    device = std::make_shared<dai::Device>(pipeline, deviceInfo);
                } else {
                    device = std::make_shared<dai::Device>(pipeline, deviceInfo, usb2Mode);
                }
                break;
            } else if(deviceInfo.state == X_LINK_BOOTED) {
                throw std::runtime_error("ros::NodeHandle() from Node \"" + pnh.getNamespace() + "\" DepthAI Device with MxId  \"" + mxId
                                         + "\" is already booted on different process.  \"");
            }
        } else if(mxId.empty()) {
            isDeviceFound = true;
            device = std::make_shared<dai::Device>(pipeline);
        }
    }

    if(!isDeviceFound) {
        throw std::runtime_error("ros::NodeHandle() from Node \"" + pnh.getNamespace() + "\" DepthAI Device with MxId  \"" + mxId + "\" not found.  \"");
    }

    if(!poeMode) {
        std::cout << "Device USB status: " << usbStrings[static_cast<int32_t>(device->getUsbSpeed())] << std::endl;
    }

    // Apply camera controls
    auto controlQueue = device->getInputQueue("control");

    // Set manual exposure
    if(manualExposure) {
        dai::CameraControl ctrl;
        ctrl.setManualExposure(expTime, sensIso);
        controlQueue->send(ctrl);
    }

    std::shared_ptr<dai::DataOutputQueue> stereoQueue;
    if(enableDepth) {
        stereoQueue = device->getOutputQueue("depth", 30, false);
    } else {
        stereoQueue = device->getOutputQueue("disparity", 30, false);
    }
    auto imuQueue = device->getOutputQueue("imu", 30, false);

    auto calibrationHandler = device->readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    if(height > 480 && boardName == "OAK-D-LITE" && depth_aligned == false) {
        width = 640;
        height = 480;
    }
    std::vector<std::tuple<std::string, int, int>> irDrivers = device->getIrDrivers();
    if(!irDrivers.empty()) {
        if(enableDotProjector) {
            device->setIrLaserDotProjectorBrightness(dotProjectormA);
        }

        if(enableFloodLight) {
            device->setIrFloodLightBrightness(floodLightmA);
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

    dai::rosBridge::BridgePublisher<sensor_msgs::Imu, dai::IMUData> imuPublish(
        imuQueue,
        pnh,
        std::string("imu"),
        std::bind(&dai::rosBridge::ImuConverter::toRosMsg, &imuConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "imu");

    imuPublish.addPublisherCallback();
    /*
    int colorWidth = 1280, colorHeight = 720;
    if(height < 720) {
        colorWidth = 640;
        colorHeight = 360;
    }
    */

    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    if(enableRosBaseTimeUpdate) {
        rgbConverter.setUpdateRosBaseTimeOnToRosMsg();
    }
    if(enableDepth) {
        auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, width, height);

        auto depthCameraInfo =
            depth_aligned ? rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, width, height) : rightCameraInfo;

        auto depthconverter = depth_aligned ? rgbConverter : rightconverter;
        dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(
            stereoQueue,
            pnh,
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
            auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, width, height);
            auto imgQueue = device->getOutputQueue("rgb", 30, false);
            dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(
                imgQueue,
                pnh,
                std::string("color/image"),
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
                30,
                rgbCameraInfo,
                "color");
            rgbPublish.addPublisherCallback();

            if(enableSpatialDetection) {
                auto previewQueue = device->getOutputQueue("preview", 30, false);
                auto detectionQueue = device->getOutputQueue("detections", 30, false);
                auto previewCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, previewWidth, previewHeight);

                dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> previewPublish(
                    previewQueue,
                    pnh,
                    std::string("color/preview/image"),
                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
                    30,
                    previewCameraInfo,
                    "color/preview");
                previewPublish.addPublisherCallback();

                dai::rosBridge::SpatialDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", 416, 416, false);
                dai::rosBridge::BridgePublisher<depthai_ros_msgs::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(
                    detectionQueue,
                    pnh,
                    std::string("color/yolov4_Spatial_detections"),
                    std::bind(&dai::rosBridge::SpatialDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
                    30);
                detectionPublish.addPublisherCallback();
                ros::spin();
            }

            ros::spin();
        } else {
            auto leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, width, height);
            auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, width, height);

            auto leftQueue = device->getOutputQueue("left", 30, false);
            auto rightQueue = device->getOutputQueue("right", 30, false);
            dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> leftPublish(
                leftQueue,
                pnh,
                leftPubName,
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &converter, std::placeholders::_1, std::placeholders::_2),
                30,
                leftCameraInfo,
                "left");
            dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rightPublish(
                rightQueue,
                pnh,
                rightPubName,
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rightconverter, std::placeholders::_1, std::placeholders::_2),
                30,
                rightCameraInfo,
                "right");
            rightPublish.addPublisherCallback();
            leftPublish.addPublisherCallback();
            ros::spin();
        }
    } else {
        std::string tfSuffix = depth_aligned ? "_rgb_camera_optical_frame" : "_right_camera_optical_frame";
        dai::rosBridge::DisparityConverter dispConverter(tfPrefix + tfSuffix, 880, 7.5, 20, 2000);  // TODO(sachin): undo hardcoding of baseline

        auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, width, height);
        auto disparityCameraInfo =
            depth_aligned ? rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, width, height) : rightCameraInfo;
        auto depthconverter = depth_aligned ? rgbConverter : rightconverter;
        dai::rosBridge::BridgePublisher<stereo_msgs::DisparityImage, dai::ImgFrame> dispPublish(
            stereoQueue,
            pnh,
            std::string("stereo/disparity"),
            std::bind(&dai::rosBridge::DisparityConverter::toRosMsg, &dispConverter, std::placeholders::_1, std::placeholders::_2),
            30,
            disparityCameraInfo,
            "stereo");
        dispPublish.addPublisherCallback();

        if(depth_aligned) {
            auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, width, height);
            auto imgQueue = device->getOutputQueue("rgb", 30, false);
            dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
            dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(
                imgQueue,
                pnh,
                std::string("color/image"),
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
                30,
                rgbCameraInfo,
                "color");
            rgbPublish.addPublisherCallback();

            if(enableSpatialDetection) {
                auto previewQueue = device->getOutputQueue("preview", 30, false);
                auto detectionQueue = device->getOutputQueue("detections", 30, false);
                auto previewCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, previewWidth, previewHeight);

                dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> previewPublish(
                    previewQueue,
                    pnh,
                    std::string("color/preview/image"),
                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rgbConverter, std::placeholders::_1, std::placeholders::_2),
                    30,
                    previewCameraInfo,
                    "color/preview");
                previewPublish.addPublisherCallback();

                dai::rosBridge::SpatialDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", 416, 416, false);
                dai::rosBridge::BridgePublisher<depthai_ros_msgs::SpatialDetectionArray, dai::SpatialImgDetections> detectionPublish(
                    detectionQueue,
                    pnh,
                    std::string("color/yolov4_Spatial_detections"),
                    std::bind(&dai::rosBridge::SpatialDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
                    30);
                detectionPublish.addPublisherCallback();
                ros::spin();
            }

            ros::spin();
        } else {
            auto leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, width, height);
            auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, width, height);

            auto leftQueue = device->getOutputQueue("left", 30, false);
            auto rightQueue = device->getOutputQueue("right", 30, false);
            dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> leftPublish(
                leftQueue,
                pnh,
                leftPubName,
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &converter, std::placeholders::_1, std::placeholders::_2),
                30,
                leftCameraInfo,
                "left");
            dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rightPublish(
                rightQueue,
                pnh,
                rightPubName,
                std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &rightconverter, std::placeholders::_1, std::placeholders::_2),
                30,
                rightCameraInfo,
                "right");
            rightPublish.addPublisherCallback();
            leftPublish.addPublisherCallback();
            ros::spin();
        }
    }

    return 0;
}
