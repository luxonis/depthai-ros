
#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <cstdio>

#include <sensor_msgs/msg/image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/ImgDetectionConverter.hpp>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

dai::Pipeline createPipeline(bool syncNN, std::string nnPath){
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
    if(syncNN) detectionNetwork->passthrough.link(xlinkOut->input);
    else colorCam->preview.link(xlinkOut->input);

    detectionNetwork->out.link(nnOut->input);
    return pipeline;
}

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mobilenet_node");
    
    std::string tfPrefix;
    std::string cameraParamUri = "package://depthai_examples/params/camera";
    std::string nnPath(BLOB_PATH);
    bool syncNN;
    int bad_params = 0;
    
    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("camera_param_uri", cameraParamUri);
    node->declare_parameter("sync_nn", true);
    node->declare_parameter<std::string>("nn_path", "");
    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("camera_param_uri", cameraParamUri);
    node->get_parameter("sync_nn", syncNN);

    // Uses the path from param if passed or else uses from BLOB_PATH from CMAKE
    std::string nnParam;
    node->get_parameter("nn_path", nnParam);
    if(!nnParam.empty())
    {   
        node->get_parameter("nn_path", nnPath);
    }

    dai::Pipeline pipeline = createPipeline(syncNN, nnPath);
    dai::Device device(pipeline);
    
    std::shared_ptr<dai::DataOutputQueue> previewQueue = device.getOutputQueue("preview", 30, false);
    std::shared_ptr<dai::DataOutputQueue> nNetDataQueue = device.getOutputQueue("detections", 30, false);

    std::string color_uri = cameraParamUri + "/" + "color.yaml";

    //TODO(sachin): Add option to use CameraInfo from EEPROM
    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(previewQueue,
                                                                                   node, 
                                                                                   std::string("color/image"),
                                                                                   std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                   &rgbConverter, // since the converter has the same frame name
                                                                                                   // and image type is also same we can reuse it
                                                                                   std::placeholders::_1, 
                                                                                   std::placeholders::_2) , 
                                                                                   30,
                                                                                   color_uri,
                                                                                   "color");


    dai::rosBridge::ImgDetectionConverter detConverter(tfPrefix + "_rgb_camera_optical_frame", 300, 300, false);
    dai::rosBridge::BridgePublisher<vision_msgs::msg::Detection2DArray, dai::ImgDetections> detectionPublish(nNetDataQueue,
                                                                                                         node, 
                                                                                                         std::string("color/mobilenet_detections"),
                                                                                                         std::bind(static_cast<void(dai::rosBridge::ImgDetectionConverter::*)(std::shared_ptr<dai::ImgDetections>, 
                                                                                                         vision_msgs::msg::Detection2DArray&)>(&dai::rosBridge::ImgDetectionConverter::toRosMsg), 
                                                                                                         &detConverter,
                                                                                                         std::placeholders::_1, 
                                                                                                         std::placeholders::_2), 
                                                                                                         30);

    detectionPublish.addPublisherCallback();
    rgbPublish.addPublisherCallback(); // addPublisherCallback works only when the dataqueue is non blocking.

    rclcpp::spin(node);

    return 0;
}

