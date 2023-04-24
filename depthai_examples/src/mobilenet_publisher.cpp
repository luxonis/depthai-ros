

#include <cstdio>
#include <iostream>

#include "camera_info_manager/camera_info_manager.h"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "ros/node_handle.h"
#include "sensor_msgs/Image.h"
#include "vision_msgs/Detection2DArray.h"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"

dai::Pipeline createPipeline(bool syncNN, std::string nnPath) {
    dai::Pipeline pipeline;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    auto detectionNetwork = pipeline.create<dai::node::MobileNetDetectionNetwork>();
    auto nnOut = pipeline.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("preview");
    nnOut->setStreamName("detections");

    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->setFps(40);

    // testing MobileNet DetectionNetwork
    detectionNetwork->setConfidenceThreshold(0.5f);
    detectionNetwork->setBlobPath(nnPath);

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(detectionNetwork->input);
    if(syncNN)
        detectionNetwork->passthrough.link(xlinkOut->input);
    else
        colorCam->preview.link(xlinkOut->input);

    detectionNetwork->out.link(nnOut->input);
    return pipeline;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mobilenet_node");
    ros::NodeHandle pnh("~");

    std::string tfPrefix;
    std::string cameraParamUri, resourceBaseFolder, nnPath;
    std::string nnName(BLOB_NAME);
    bool syncNN;
    int badParams = 0;

    badParams += !pnh.getParam("tf_prefix", tfPrefix);
    badParams += !pnh.getParam("camera_param_uri", cameraParamUri);
    badParams += !pnh.getParam("sync_nn", syncNN);
    badParams += !pnh.getParam("resourceBaseFolder", resourceBaseFolder);

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
    dai::Pipeline pipeline = createPipeline(syncNN, nnPath);
    dai::Device device(pipeline);

    std::shared_ptr<dai::DataOutputQueue> previewQueue = device.getOutputQueue("preview", 30, false);
    std::shared_ptr<dai::DataOutputQueue> nNetDataQueue = device.getOutputQueue("detections", 30, false);

    std::string color_uri = cameraParamUri + "/" + "color.yaml";

    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(previewQueue,
                                                                                  pnh,
                                                                                  std::string("color/image"),
                                                                                  std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                            &rgbConverter,  // since the converter has the same frame name
                                                                                                            // and image type is also same we can reuse it
                                                                                            std::placeholders::_1,
                                                                                            std::placeholders::_2),
                                                                                  30,
                                                                                  color_uri,
                                                                                  "color");

    dai::rosBridge::ImgDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", 300, 300, false);
    dai::rosBridge::BridgePublisher<vision_msgs::Detection2DArray, dai::ImgDetections> detectionPublish(
        nNetDataQueue,
        pnh,
        std::string("color/mobilenet_detections"),
        std::bind(&dai::rosBridge::ImgDetectionConverter::toRosMsg, &detConverter, std::placeholders::_1, std::placeholders::_2),
        30);

    detectionPublish.addPublisherCallback();
    rgbPublish.addPublisherCallback();  // addPublisherCallback works only when the dataqueue is non blocking.

    ros::spin();

    return 0;
}
