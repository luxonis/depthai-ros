
#include "ros/ros.h"

#include <iostream>
#include <cstdio>
// #include "utility.hpp"
#include <depthai_examples/daiUtility.hpp>
#include <depthai_examples/nn_pipeline.hpp>

#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include <vision_msgs/Detection2DArray.h>

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/DetectionConverter.hpp>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"


int main(int argc, char** argv){

    ros::init(argc, argv, "mobilenet_node");
    ros::NodeHandle pnh("~");
    
    std::string deviceName;
    std::string camera_param_uri;
    std::string nnPath(BLOB_PATH);
    int bad_params = 0;

    bad_params += !pnh.getParam("camera_name", deviceName);
    // bad_params += !pnh.getParam("nnPath", nnPath);
    bad_params += !pnh.getParam("camera_param_uri", camera_param_uri);

    if (bad_params > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }

    MobileNetDetectionExample detectionPipeline;
    detectionPipeline.initDepthaiDev(nnPath);
    std::vector<std::shared_ptr<dai::DataOutputQueue>> imageDataQueues = detectionPipeline.getExposedImageStreams();
    std::vector<std::shared_ptr<dai::DataOutputQueue>> nNetDataQueues = detectionPipeline.getExposedNnetStreams();;

    // std::vector<ros::Publisher> imgPubList;
    // std::vector<ros::Publisher> nNetPubList;
    // std::vector<std::string> frameNames;

    std::string color_uri = camera_param_uri + "/" + "color.yaml";

    dai::rosBridge::ImageConverter rgbConverter(deviceName + "_rgb_camera_optical_frame", false);
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(imageDataQueues[0],
                                                                                     pnh, 
                                                                                     std::string("color/image"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &rgbConverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     color_uri,
                                                                                     "color");


    dai::rosBridge::DetectionConverter<vision_msgs::Detection2DArray> detConverter(deviceName + "_rgb_camera_optical_frame", 300, 300, false);
    dai::rosBridge::BridgePublisher<vision_msgs::Detection2DArray, dai::ImgDetections> detectionPublish(nNetDataQueues[0],
                                                                                     pnh, 
                                                                                     std::string("color/mobilenet_detections"),
                                                                                     std::bind(static_cast<void(dai::rosBridge::DetectionConverter<vision_msgs::Detection2DArray>::*)(std::shared_ptr<dai::ImgDetections>, 
                                                                                     vision_msgs::Detection2DArray&)>(&dai::rosBridge::DetectionConverter<vision_msgs::Detection2DArray>::toRosMsg), 
                                                                                     &detConverter,
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30);

    // rgbPublish.startPublisherThread();
    detectionPublish.startPublisherThread(); // addPubisherCallback works only when the dataqueue is non blocking.
    rgbPublish.addPubisherCallback();
    // detectionPublish.addPubisherCallback();

    ros::spin();

    return 0;
}

