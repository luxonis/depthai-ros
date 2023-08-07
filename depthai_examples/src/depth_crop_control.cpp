/**
 * This example shows usage of depth camera in crop mode with the possibility to move the crop.
 * Use 'WASD' in order to do it.
 */

#include <iostream>
#include <memory>
#include <string>

#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_msgs/NormalizedImageCrop.h"
#include "ros/node_handle.h"
#include "sensor_msgs/Image.h"

// Step size ('W','A','S','D' controls)
static constexpr float stepSize = 0.02;
std::shared_ptr<dai::DataInputQueue> configQueue;

bool cropDepthImage(depthai_ros_msgs::NormalizedImageCrop::Request request, depthai_ros_msgs::NormalizedImageCrop::Response response) {
    dai::ImageManipConfig cfg;
    cfg.setCropRect(request.topLeft.x, request.topLeft.y, request.bottomRight.x, request.bottomRight.y);
    configQueue->send(cfg);
    return true;
}

int main() {
    ros::init(argc, argv, "depth_crop_control");
    ros::NodeHandle pnh("~");
    std::string cameraName;
    std::string monoResolution = "720p";
    int confidence = 200;
    int LRchecktresh = 5;

    int badParams = 0;
    badParams += !pnh.getParam("tf_prefix", cameraName);
    badParams += !pnh.getParam("lrcheck", lrcheck);
    badParams += !pnh.getParam("extended", extended);
    badParams += !pnh.getParam("subpixel", subpixel);
    badParams += !pnh.getParam("confidence", confidence);
    badParams += !pnh.getParam("LRchecktresh", LRchecktresh);

    badParams += !pnh.getParam("monoResolution", monoResolution);

    if(badParams > 0) {
        std::cout << " Bad parameters -> " << badParams << std::endl;
        throw std::runtime_error("Couldn't find the parameters", );
    }

    ros::ServiceServer service = n.advertiseService("crop_control_srv", cropDepthImage);

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto manip = pipeline.create<dai::node::ImageManip>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto configIn = pipeline.create<dai::node::XLinkIn>();
    auto xout = pipeline.create<dai::node::XLinkOut>();

    configIn->setStreamName("config");
    xout->setStreamName("depth");

    // Crop range
    dai::Point2f topLeft(0.2, 0.2);
    dai::Point2f bottomRight(0.8, 0.8);

    dai::node::MonoCamera::Properties::SensorResolution monoRes;
    int monoWidth, monoHeight;
    if(monoResolution == "720p") {
        monoRes = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
        monoWidth = 1280;
        monoHeight = 720;
    } else if(monoResolution == "400p") {
        monoRes = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
        monoWidth = 640;
        monoHeight = 400;
    } else if(monoResolution == "800p") {
        monoRes = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
        monoWidth = 1280;
        monoHeight = 800;
    } else if(monoResolution == "480p") {
        monoRes = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
        monoWidth = 640;
        monoHeight = 480;
    } else {
        ROS_ERROR("Invalid parameter. -> monoResolution: %s", monoResolution.c_str());
        throw std::runtime_error("Invalid mono camera resolution.");
    }

    // Properties
    monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoRight->setResolution(monoRes);
    monoLeft->setResolution(monoRes);

    manip->initialConfig.setCropRect(topLeft.x, topLeft.y, bottomRight.x, bottomRight.y);
    manip->setMaxOutputFrameSize(monoRight->getResolutionHeight() * monoRight->getResolutionWidth() * 3);
    stereo->initialConfig.setConfidenceThreshold(confidence);
    stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    // Linking
    configIn->out.link(manip->inputConfig);
    stereo->depth.link(manip->inputImage);
    manip->out.link(xout->input);
    monoRight->out.link(stereo->left);
    monoLeft->out.link(stereo->right);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Queues
    auto depthQueue = device.getOutputQueue(xout->getStreamName(), 5, false);
    configQueue = device.getInputQueue(configIn->getStreamName());

    auto calibrationHandler = device.readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    if(monoHeight > 480 && boardName == "OAK-D-LITE") {
        monoWidth = 640;
        monoHeight = 480;
    }

    dai::rosBridge::ImageConverter depthConverter(cameraName + "_right_camera_optical_frame", true);
    // TODO(sachin): Modify the calibration based on crop from service
    auto rightCameraInfo = converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);

    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(depthQueue,
                                                                                    pnh,
                                                                                    std::string("stereo/depth"),
                                                                                    std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                              &depthConverter,  // since the converter has the same frame name
                                                                                                                // and image type is also same we can reuse it
                                                                                              std::placeholders::_1,
                                                                                              std::placeholders::_2),
                                                                                    30,
                                                                                    rightCameraInfo,
                                                                                    "stereo");
    depthPublish.addPublisherCallback();
    ros::spin();

    return 0;
}
