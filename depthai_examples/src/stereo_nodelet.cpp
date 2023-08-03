#include <functional>
#include <tuple>

#include "camera_info_manager/camera_info_manager.h"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"
#include "ros/node_handle.h"
#include "sensor_msgs/Image.h"

// Inludes common necessary includes for development using depthai library

#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"

namespace depthai_examples {

class StereoNodelet : public nodelet::Nodelet {
    std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>> leftPublish, rightPublish, depthPublish;
    std::unique_ptr<dai::rosBridge::ImageConverter> leftConverter, rightConverter;
    std::unique_ptr<dai::Device> _dev;

   public:
    virtual void onInit() override {
        auto& pnh = getPrivateNodeHandle();

        std::string tfPrefix, mode;
        std::string cameraParamUri;
        std::string monoResolution = "720p";
        int badParams = 0;
        bool lrcheck, extended, subpixel, enableDepth;
        int confidence = 200;
        int LRchecktresh = 5;

        badParams += !pnh.getParam("tf_prefix", tfPrefix);
        badParams += !pnh.getParam("camera_param_uri", cameraParamUri);
        badParams += !pnh.getParam("mode", mode);
        badParams += !pnh.getParam("lrcheck", lrcheck);
        badParams += !pnh.getParam("extended", extended);
        badParams += !pnh.getParam("subpixel", subpixel);
        badParams += !pnh.getParam("confidence", confidence);
        badParams += !pnh.getParam("LRchecktresh", LRchecktresh);
        badParams += !pnh.getParam("monoResolution", monoResolution);

        if(badParams > 0) {
            std::cout << " Bad parameters -> " << badParams << std::endl;
            throw std::runtime_error("Couldn't find %d of the parameters");
        }

        if(mode == "depth") {
            enableDepth = true;
        } else {
            enableDepth = false;
        }

        dai::Pipeline pipeline;
        int monoWidth, monoHeight;
        std::tie(pipeline, monoWidth, monoHeight) = createPipeline(enableDepth, lrcheck, extended, subpixel, confidence, LRchecktresh, monoResolution);
        _dev = std::make_unique<dai::Device>(pipeline);

        auto leftQueue = _dev->getOutputQueue("left", 30, false);
        auto rightQueue = _dev->getOutputQueue("right", 30, false);

        std::shared_ptr<dai::DataOutputQueue> stereoQueue;
        if(enableDepth) {
            stereoQueue = _dev->getOutputQueue("depth", 30, false);
        } else {
            stereoQueue = _dev->getOutputQueue("disparity", 30, false);
        }
        auto calibrationHandler = _dev->readCalibration();

        // this part would be removed once we have calibration-api
        /*
        std::string left_uri = camera_param_uri +"/" + "left.yaml";

        std::string right_uri = camera_param_uri + "/" + "right.yaml";

        std::string stereo_uri = camera_param_uri + "/" + "right.yaml";
        */

        auto boardName = calibrationHandler.getEepromData().boardName;
        if(monoHeight > 480 && boardName == "OAK-D-LITE") {
            monoWidth = 640;
            monoHeight = 480;
        }

        leftConverter = std::make_unique<dai::rosBridge::ImageConverter>(tfPrefix + "_left_camera_optical_frame", true);
        auto leftCameraInfo = leftConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_B, monoWidth, monoHeight);

        leftPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(
            leftQueue,
            pnh,
            std::string("left/image"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, leftConverter.get(), std::placeholders::_1, std::placeholders::_2),
            30,
            leftCameraInfo,
            "left");

        // bridgePublish.startPublisherThread();
        leftPublish->addPublisherCallback();

        rightConverter = std::make_unique<dai::rosBridge::ImageConverter>(tfPrefix + "_right_camera_optical_frame", true);
        auto rightCameraInfo = rightConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_C, monoWidth, monoHeight);

        rightPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(
            rightQueue,
            pnh,
            std::string("right/image"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg, rightConverter.get(), std::placeholders::_1, std::placeholders::_2),
            30,
            rightCameraInfo,
            "right");

        rightPublish->addPublisherCallback();

        // dai::rosBridge::ImageConverter depthConverter(tfPrefix + "_right_camera_optical_frame");
        depthPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>(
            stereoQueue,
            pnh,
            std::string("stereo/depth"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                      rightConverter.get(),  // since the converter has the same frame name
                                             // and image type is also same we can reuse it
                      std::placeholders::_1,
                      std::placeholders::_2),
            30,
            rightCameraInfo,
            "stereo");

        depthPublish->addPublisherCallback();

        // We can add the rectified frames also similar to these publishers.
        // Left them out so that users can play with it by adding and removing
    }

    std::tuple<dai::Pipeline, int, int> createPipeline(
        bool withDepth, bool lrcheck, bool extended, bool subpixel, int confidence, int LRchecktresh, std::string resolution) {
        dai::Pipeline pipeline;

        auto monoLeft = pipeline.create<dai::node::MonoCamera>();
        auto monoRight = pipeline.create<dai::node::MonoCamera>();
        auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
        auto xoutRight = pipeline.create<dai::node::XLinkOut>();
        auto stereo = pipeline.create<dai::node::StereoDepth>();
        auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

        // XLinkOut
        xoutLeft->setStreamName("left");
        xoutRight->setStreamName("right");

        if(withDepth) {
            xoutDepth->setStreamName("depth");
        } else {
            xoutDepth->setStreamName("disparity");
        }

        int width, height;
        dai::node::MonoCamera::Properties::SensorResolution monoResolution;
        if(resolution == "720p") {
            monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P;
            width = 1280;
            height = 720;
        } else if(resolution == "400p") {
            monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P;
            width = 640;
            height = 400;
        } else if(resolution == "800p") {
            monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P;
            width = 1280;
            height = 800;
        } else if(resolution == "480p") {
            monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P;
            width = 640;
            height = 480;
        } else {
            ROS_ERROR("Invalid parameter. -> monoResolution: %s", resolution.c_str());
            throw std::runtime_error("Invalid mono camera resolution.");
        }

        // MonoCamera
        monoLeft->setResolution(monoResolution);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::CAM_B);
        monoRight->setResolution(monoResolution);
        monoRight->setBoardSocket(dai::CameraBoardSocket::CAM_C);

        // int maxDisp = 96;
        // if (extended) maxDisp *= 2;
        // if (subpixel) maxDisp *= 32; // 5 bits fractional disparity

        // StereoDepth
        stereo->initialConfig.setConfidenceThreshold(confidence);
        stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
        stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout

        stereo->setLeftRightCheck(lrcheck);
        stereo->setExtendedDisparity(extended);
        stereo->setSubpixel(subpixel);

        // Link plugins CAM -> STEREO -> XLINK
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);

        stereo->syncedLeft.link(xoutLeft->input);
        stereo->syncedRight.link(xoutRight->input);

        if(withDepth) {
            stereo->depth.link(xoutDepth->input);
        } else {
            stereo->disparity.link(xoutDepth->input);
        }

        return std::make_tuple(pipeline, width, height);
    }
};

PLUGINLIB_EXPORT_CLASS(depthai_examples::StereoNodelet, nodelet::Nodelet)
}  // namespace depthai_examples
