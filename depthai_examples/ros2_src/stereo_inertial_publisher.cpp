
#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <cstdio>
#include <tuple>
#include "sensor_msgs/msg/imu.h"
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <functional>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImuConverter.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/DisparityConverter.hpp>


std::tuple<dai::Pipeline, int, int> createPipeline(bool enableDepth, bool lrcheck, bool extended, bool subpixel, bool rectify, bool depth_aligned, int stereo_fps, int confidence, int LRchecktresh, std::string resolution){
    dai::Pipeline pipeline;

    auto monoLeft  = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo    = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto imu       = pipeline.create<dai::node::IMU>();
    auto xoutImu   = pipeline.create<dai::node::XLinkOut>();

    if (enableDepth) {
        xoutDepth->setStreamName("depth");
    }
    else {
        xoutDepth->setStreamName("disparity");
    }

    xoutImu->setStreamName("imu");

    int width, height;
    dai::node::MonoCamera::Properties::SensorResolution monoResolution; 
    if(resolution == "720p"){
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P; 
        width  = 1280;
        height = 720;
    }else if(resolution == "400p" ){
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P; 
        width  = 640;
        height = 400;
    }else if(resolution == "800p" ){
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P; 
        width  = 1280;
        height = 800;
    }else if(resolution == "480p" ){
        monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P; 
        width  = 640;
        height = 480;
    }else{
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                    "Invalid parameter. -> monoResolution: %s", resolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }
    // MonoCamera
    monoLeft->setResolution(monoResolution);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoLeft->setFps(stereo_fps);
    monoRight->setResolution(monoResolution);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoRight->setFps(stereo_fps);

    // StereoDepth
    stereo->initialConfig.setConfidenceThreshold(confidence); //Known to be best
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh); //Known to be best
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);
    if(enableDepth && depth_aligned) stereo->setDepthAlign(dai::CameraBoardSocket::RGB);

    //Imu
    imu->enableIMUSensor({dai::IMUSensor::ROTATION_VECTOR, dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 400);
    imu->setMaxBatchReports(1); // Get one message only for now.

    if(enableDepth && depth_aligned){
        // RGB image
        auto camRgb               = pipeline.create<dai::node::ColorCamera>();
        auto xoutRgb              = pipeline.create<dai::node::XLinkOut>();
        xoutRgb->setStreamName("rgb");
        camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
        camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        // the ColorCamera is downscaled from 1080p to 720p.
        // Otherwise, the aligned depth is automatically upscaled to 1080p
        camRgb->setIspScale(2, 3);
        // For now, RGB needs fixed focus to properly align with depth.
        // This value was used during calibration
        camRgb->initialControl.setManualFocus(135);
        camRgb->isp.link(xoutRgb->input);
    }else{
        // Stereo imges
        auto xoutLeft             = pipeline.create<dai::node::XLinkOut>();
        auto xoutRight            = pipeline.create<dai::node::XLinkOut>();
        // XLinkOut
        xoutLeft->setStreamName("left");
        xoutRight->setStreamName("right");
        if(rectify){
            stereo->rectifiedLeft.link(xoutLeft->input);
            stereo->rectifiedRight.link(xoutRight->input);     
        }else{
            stereo->syncedLeft.link(xoutLeft->input);
            stereo->syncedRight.link(xoutRight->input);
        }
    }

    // Link plugins CAM -> STEREO -> XLINK

    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    if(enableDepth){
        stereo->depth.link(xoutDepth->input);
    }
    else{
        stereo->disparity.link(xoutDepth->input);
    }

    imu->out.link(xoutImu->input);

    return std::make_tuple(pipeline, width, height);
}

int main(int argc, char** argv){
    
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_inertial_node");

    std::string tfPrefix, mode, monoResolution;
    int badParams = 0, stereo_fps, confidence, LRchecktresh;
    bool lrcheck, extended, subpixel, enableDepth, rectify, depth_aligned;

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("mode", "depth");
    node->declare_parameter("lrcheck",  true);
    node->declare_parameter("extended",  false);
    node->declare_parameter("subpixel",  true);
    node->declare_parameter("rectify",  false);
    node->declare_parameter("depth_aligned",  false);
    node->declare_parameter("stereo_fps",  30);
    node->declare_parameter("confidence",  200);
    node->declare_parameter("LRchecktresh",  5);
    node->declare_parameter("monoResolution",  "720p");

    node->get_parameter("tf_prefix",     tfPrefix);
    node->get_parameter("mode",          mode);
    node->get_parameter("lrcheck",       lrcheck);
    node->get_parameter("extended",      extended);
    node->get_parameter("subpixel",      subpixel);
    node->get_parameter("rectify",       rectify);
    node->get_parameter("depth_aligned", depth_aligned);
    node->get_parameter("stereo_fps",    stereo_fps);
    node->get_parameter("confidence",    confidence);
    node->get_parameter("LRchecktresh",  LRchecktresh);
    node->get_parameter("monoResolution", monoResolution);

    if(mode == "depth"){
        enableDepth = true;
    }
    else{
        enableDepth = false;
    }

    dai::Pipeline pipeline;
    int monoWidth, monoHeight;
    std::tie(pipeline, monoWidth, monoHeight) = createPipeline(enableDepth, lrcheck, extended, subpixel, rectify, depth_aligned, stereo_fps, confidence, LRchecktresh, monoResolution);

    dai::Device device(pipeline);

    std::shared_ptr<dai::DataOutputQueue> stereoQueue;
    if (enableDepth) {
        stereoQueue = device.getOutputQueue("depth", 30, false);
    }else{
        stereoQueue = device.getOutputQueue("disparity", 30, false);
    }
    auto imuQueue = device.getOutputQueue("imu",30,false);

    auto calibrationHandler = device.readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    if (monoHeight > 480 && boardName == "OAK-D-LITE") {
        monoWidth = 640;
        monoHeight = 480;
    }
    
    dai::rosBridge::ImageConverter converter(tfPrefix + "_left_camera_optical_frame", true);
    dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", true);
    auto leftCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, monoWidth, monoHeight); 
    auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, monoWidth, monoHeight); 
    const std::string leftPubName = rectify?std::string("left/image_rect"):std::string("left/image_raw");
    const std::string rightPubName = rectify?std::string("right/image_rect"):std::string("right/image_raw");

    dai::rosBridge::ImuConverter imuConverter(tfPrefix +"_imu_frame");

    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData> ImuPublish(imuQueue,
                                                                                     node, 
                                                                                     std::string("imu"),
                                                                                     std::bind(&dai::rosBridge::ImuConverter::toRosMsg, 
                                                                                     &imuConverter, 
                                                                                     std::placeholders::_1,
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     "",
                                                                                     "imu");

    ImuPublish.addPublisherCallback();

    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, 1280, 720);
    
     if(enableDepth){
        std::cout << "In depth----------------------------------";
        auto depthCameraInfo = depth_aligned ? rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, 1280, 720) : rightCameraInfo;
        auto depthconverter = depth_aligned ? rgbConverter : rightconverter;
        dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(stereoQueue,
                                                                                     node, 
                                                                                     std::string("stereo/depth"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &depthconverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     depthCameraInfo,
                                                                                     "stereo");
        depthPublish.addPublisherCallback();
        
        if(depth_aligned){
            auto imgQueue = device.getOutputQueue("rgb", 30, false);
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(imgQueue,
                                                                                        node, 
                                                                                        std::string("color/image"),
                                                                                        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                        &rgbConverter,
                                                                                        std::placeholders::_1, 
                                                                                        std::placeholders::_2) , 
                                                                                        30,
                                                                                        rgbCameraInfo,
                                                                                        "color");
            rgbPublish.addPublisherCallback();
            rclcpp::spin(node);
        }
        else {
            auto leftQueue = device.getOutputQueue("left", 30, false);
            auto rightQueue = device.getOutputQueue("right", 30, false);
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftPublish(leftQueue,
                                                                                            node, 
                                                                                            leftPubName,
                                                                                            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                            &converter, 
                                                                                            std::placeholders::_1, 
                                                                                            std::placeholders::_2) , 
                                                                                            30,
                                                                                            leftCameraInfo,
                                                                                            "left");
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(rightQueue,
                                                                                            node, 
                                                                                            rightPubName,
                                                                                            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                            &rightconverter, 
                                                                                            std::placeholders::_1, 
                                                                                            std::placeholders::_2) , 
                                                                                            30,
                                                                                            rightCameraInfo,
                                                                                            "right");  
            rightPublish.addPublisherCallback();
            leftPublish.addPublisherCallback();
            rclcpp::spin(node);
        }
    }
    else{
        std::string tfSuffix = depth_aligned ? "_rgb_camera_optical_frame" : "_right_camera_optical_frame";
        dai::rosBridge::DisparityConverter dispConverter(tfPrefix + tfSuffix , 880, 7.5, 20, 2000); // TODO(sachin): undo hardcoding of baseline
        auto disparityCameraInfo = depth_aligned ? rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, 1280, 720) : rightCameraInfo;
        auto depthconverter = depth_aligned ? rgbConverter : rightconverter;
        dai::rosBridge::BridgePublisher<stereo_msgs::msg::DisparityImage, dai::ImgFrame> dispPublish(stereoQueue,
                                                                                     node, 
                                                                                     std::string("stereo/disparity"),
                                                                                     std::bind(&dai::rosBridge::DisparityConverter::toRosMsg, 
                                                                                     &dispConverter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     disparityCameraInfo,
                                                                                     "stereo");
        dispPublish.addPublisherCallback();
        if(depth_aligned){
            auto imgQueue = device.getOutputQueue("rgb", 30, false);
            dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(imgQueue,
                                                                                        node, 
                                                                                        std::string("color/image"),
                                                                                        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                        &rgbConverter,
                                                                                        std::placeholders::_1, 
                                                                                        std::placeholders::_2) , 
                                                                                        30,
                                                                                        rgbCameraInfo,
                                                                                        "color");
            rgbPublish.addPublisherCallback();
            rclcpp::spin(node);
        }
        else {
            auto leftQueue = device.getOutputQueue("left", 30, false);
            auto rightQueue = device.getOutputQueue("right", 30, false);
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> leftPublish(leftQueue,
                                                                                            node, 
                                                                                            leftPubName,
                                                                                            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                            &converter, 
                                                                                            std::placeholders::_1, 
                                                                                            std::placeholders::_2) , 
                                                                                            30,
                                                                                            leftCameraInfo,
                                                                                            "left");
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(rightQueue,
                                                                                            node, 
                                                                                            rightPubName,
                                                                                            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                            &rightconverter, 
                                                                                            std::placeholders::_1, 
                                                                                            std::placeholders::_2) , 
                                                                                            30,
                                                                                            rightCameraInfo,
                                                                                            "right");  
            rightPublish.addPublisherCallback();
            leftPublish.addPublisherCallback();
            rclcpp::spin(node);
        }
    }
    
    return 0;
}
