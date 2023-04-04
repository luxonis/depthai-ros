
#include <chrono>
#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"

// #include "utility.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "sensor_msgs/msg/image.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"

using namespace std::chrono_literals;

std::tuple<dai::Pipeline, int, int> createPipeline(bool lrcheck,
                                                   bool extended,
                                                   bool subpixel,
                                                   int confidence,
                                                   int LRchecktresh,
                                                   bool useVideo,
                                                   bool usePreview,
                                                   int previewWidth,
                                                   int previewHeight,
                                                   std::string mResolution,
                                                   std::string cResolution) {
    dai::Pipeline pipeline;
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();

    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

    xoutDepth->setStreamName("depth");

    int width, height;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution;
    if(mResolution == "720p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
        width = 1280;
        height = 720;
    } else if(mResolution == "400p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
        width = 640;
        height = 400;
    } else if(mResolution == "800p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
        width = 1280;
        height = 800;
    } else if(mResolution == "480p") {
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
        width = 640;
        height = 480;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> monoResolution: %s", mResolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    // MonoCamera
    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

    // StereoDepth
    stereo->initialConfig.setConfidenceThreshold(confidence);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    // // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    stereo->depth.link(xoutDepth->input);

    // Color camers steream setup -------->
    auto colorCam = pipeline.create<dai::node::ColorCamera>();

    dai::ColorCameraProperties::SensorResolution colorResolution;
    if(cResolution == "1080p") {
        colorResolution = dai::ColorCameraProperties::SensorResolution::THE_1080_P;
    } else if(cResolution == "4K") {
        colorResolution = dai::ColorCameraProperties::SensorResolution::THE_4_K;
    }

    colorCam->setResolution(colorResolution);
    if(cResolution == "1080p") {
        colorCam->setVideoSize(1920, 1080);
    } else {
        colorCam->setVideoSize(3840, 2160);
    }

    colorCam->setPreviewSize(previewWidth, previewHeight);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->setFps(30);

    auto xlinkPreviewOut = pipeline.create<dai::node::XLinkOut>();
    xlinkPreviewOut->setStreamName("preview");

    if(usePreview) {
        colorCam->preview.link(xlinkPreviewOut->input);
    }

    auto xlinkVideoOut = pipeline.create<dai::node::XLinkOut>();
    xlinkVideoOut->setStreamName("video");
    xlinkVideoOut->input.setQueueSize(1);

    if(useVideo) {
        colorCam->video.link(xlinkVideoOut->input);
    }

    return std::make_tuple(pipeline, width, height);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rgb_stereo_node");

    std::string tfPrefix, monoResolution, colorResolution;
    bool lrcheck, extended, subpixel;
    bool useVideo, usePreview, useDepth;
    int confidence, LRchecktresh, previewWidth, previewHeight;
    float dotProjectormA, floodLightmA;

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("lrcheck", true);
    node->declare_parameter("extended", false);
    node->declare_parameter("subpixel", true);
    node->declare_parameter("confidence", 200);
    node->declare_parameter("LRchecktresh", 5);
    node->declare_parameter("monoResolution", "720p");
    node->declare_parameter("colorResolution", "1080p");
    node->declare_parameter("useVideo", true);
    node->declare_parameter("usePreview", false);
    node->declare_parameter("useDepth", true);
    node->declare_parameter("previewWidth", 300);
    node->declare_parameter("previewHeight", 300);
    node->declare_parameter("dotProjectormA", 0.0f);
    node->declare_parameter("floodLightmA", 0.0f);

    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("lrcheck", lrcheck);
    node->get_parameter("extended", extended);
    node->get_parameter("subpixel", subpixel);
    node->get_parameter("confidence", confidence);
    node->get_parameter("LRchecktresh", LRchecktresh);
    node->get_parameter("monoResolution", monoResolution);
    node->get_parameter("colorResolution", colorResolution);
    node->get_parameter("useVideo", useVideo);
    node->get_parameter("usePreview", usePreview);
    node->get_parameter("useDepth", useDepth);
    node->get_parameter("previewWidth", previewWidth);
    node->get_parameter("previewHeight", previewHeight);
    node->get_parameter("dotProjectormA", dotProjectormA);
    node->get_parameter("floodLightmA", floodLightmA);

    int colorWidth, colorHeight;
    if(colorResolution == "1080p") {
        colorWidth = 1920;
        colorHeight = 1080;
    } else if(colorResolution == "4K") {
        colorWidth = 3840;
        colorHeight = 2160;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid parameter. -> colorResolution: %s", colorResolution.c_str());
        throw std::runtime_error("Invalid color camera resolution.");
    }

    dai::Pipeline pipeline;
    int monoWidth, monoHeight;
    std::tie(pipeline, monoWidth, monoHeight) = createPipeline(
        lrcheck, extended, subpixel, confidence, LRchecktresh, useVideo, usePreview, previewWidth, previewHeight, monoResolution, colorResolution);
    dai::Device device(pipeline);

    auto videoQueue = device.getOutputQueue("video", 30, false);
    auto stereoQueue = device.getOutputQueue("depth", 30, false);
    auto previewQueue = device.getOutputQueue("preview", 30, false);

    auto calibrationHandler = device.readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    if(monoHeight > 480 && boardName == "OAK-D-LITE") {
        monoWidth = 640;
        monoHeight = 480;
    }

#ifdef IS_GALACTIC
    // Parameter events for OAK-D-PRO
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> dot_cb_handle, flood_cb_handle;
    auto cb = [node, &device](const rclcpp::Parameter& p) {
        if(p.get_name() == std::string("dotProjectormA")) {
            RCLCPP_INFO(node->get_logger(), "Updating Dot Projector current to %f", p.as_double());
            device.setIrLaserDotProjectorBrightness(static_cast<float>(p.as_double()));
        } else if(p.get_name() == std::string("floodLightmA")) {
            RCLCPP_INFO(node->get_logger(), "Updating Flood Light current to %f", p.as_double());
            device.setIrFloodLightBrightness(static_cast<float>(p.as_double()));
        }
    };

    if(boardName.find("PRO") != std::string::npos) {
        param_subscriber = std::make_shared<rclcpp::ParameterEventHandler>(node);

        dot_cb_handle = param_subscriber->add_parameter_callback("dotProjectormA", cb);
        flood_cb_handle = param_subscriber->add_parameter_callback("floodLightmA", cb);
    }

#else
    rclcpp::TimerBase::SharedPtr timer;
    auto cb = [node, &device, &dotProjectormA, &floodLightmA]() {
        // rclcpp::Parameter p;
        float dotProjectormATemp, floodLightmATemp;
        node->get_parameter("dotProjectormA", dotProjectormATemp);
        node->get_parameter("floodLightmA", floodLightmATemp);
        if(dotProjectormATemp != dotProjectormA) {
            dotProjectormA = dotProjectormATemp;
            RCLCPP_INFO(node->get_logger(), "Updating Dot Projector current to %f", dotProjectormA);
            device.setIrLaserDotProjectorBrightness(static_cast<float>(dotProjectormA));
        }

        if(floodLightmATemp != floodLightmA) {
            floodLightmA = floodLightmATemp;
            RCLCPP_INFO(node->get_logger(), "Updating Flood Light current to %f", floodLightmA);
            device.setIrFloodLightBrightness(static_cast<float>(floodLightmA));
        }
    };
    if(boardName.find("PRO") != std::string::npos) {
        timer = node->create_wall_timer(500ms, cb);
    }
#endif

    std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>> depthPublish, rgbPreviewPublish, rgbPublish;

    dai::rosBridge::ImageConverter depthConverter(tfPrefix + "_right_camera_optical_frame", true);
    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", true);

    auto stereoCameraInfo = depthConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, monoWidth, monoHeight);
    auto previewCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, previewWidth, previewHeight);
    auto videoCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, colorWidth, colorHeight);

    if(useDepth) {
        depthPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>>(
            stereoQueue,
            node,
            std::string("stereo/depth"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                      &depthConverter,  // since the converter has the same frame name
                                        // and image type is also same we can reuse it
                      std::placeholders::_1,
                      std::placeholders::_2),
            30,
            stereoCameraInfo,
            "stereo");
    }

    if(usePreview) {
        rgbPreviewPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>>(
            previewQueue,
            node,
            std::string("color/preview/image"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                      &rgbConverter,  // since the converter has the same frame name
                                      // and image type is also same we can reuse it
                      std::placeholders::_1,
                      std::placeholders::_2),
            30,
            previewCameraInfo,
            "color/preview");
    }

    if(useVideo) {
        rgbPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame>>(
            videoQueue,
            node,
            std::string("color/video/image"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                      &rgbConverter,  // since the converter has the same frame name
                                      // and image type is also same we can reuse it
                      std::placeholders::_1,
                      std::placeholders::_2),
            30,
            videoCameraInfo,
            "color/video");
    }

    if(useDepth) {
        depthPublish->addPublisherCallback();  // addPublisherCallback works only when the dataqueue is non blocking.
    }

    if(usePreview) {
        rgbPreviewPublish->addPublisherCallback();
    }

    if(useVideo) {
        rgbPublish->addPublisherCallback();
    }

    // We can add the rectified frames also similar to these publishers.
    // Left them out so that users can play with it by adding and removing

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
