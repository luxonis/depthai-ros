
#include "ros/ros.h"

#include <iostream>
#include <cstdio>
// #include "utility.hpp"
#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include <depthai_examples/rgb_stereo_pipeline.hpp>
#include <functional>

// #include <depthai_examples/daiUtility.hpp>
// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

// using namespace std::placeholders;
int main(int argc, char** argv){

    ros::init(argc, argv, "rgb_stereo_node");
    ros::NodeHandle pnh("~");
    
    std::string deviceName;
    std::string camera_param_uri;
    int bad_params = 0;

    bad_params += !pnh.getParam("camera_name", deviceName);
    bad_params += !pnh.getParam("camera_param_uri", camera_param_uri);

    if (bad_params > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }

    RGBStereoExampe rgbstero_pipeline;
    rgbstero_pipeline.initDepthaiDev();
    std::vector<std::shared_ptr<dai::DataOutputQueue>> imageDataQueues = rgbstero_pipeline.getExposedImageStreams();
    
    std::vector<ros::Publisher> imgPubList;
    std::vector<std::string> frameNames;
    // std::vector<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>> bridgePublishers;
    // std::vector<dai::rosBridge::ImageConverter> converters;
    
    bool latched_cam_info = true;
    // ros::Publisher leftCamInfoPub   = pnh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 30, latched_cam_info);    
    // ros::Publisher rightCamInfoPub  = pnh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 30, latched_cam_info);    
    ros::Publisher stereoCamInfoPub = pnh.advertise<sensor_msgs::CameraInfo>("stereo/camera_info", 30, latched_cam_info);    

    // this part would be removed once we have calibration-api
    // const std::string left_uri = camera_param_uri +"/" + "left.yaml";
    // std::string name = "left";
    // camera_info_manager::CameraInfoManager left_cam_manager(ros::NodeHandle{pnh, name}, name, left_uri);
    // auto left_camera_info = left_cam_manager.getCameraInfo();
    // leftCamInfoPub.publish(left_camera_info);
    // ROS_INFO("Publishing camera onfo of left camera........");

    // const std::string right_uri = camera_param_uri + "/" + "right.yaml";
    // name = "right";
    // camera_info_manager::CameraInfoManager right_cam_manager(ros::NodeHandle{pnh, name}, name, right_uri);
    // auto right_camera_info = right_cam_manager.getCameraInfo();
    // rightCamInfoPub.publish(right_camera_info);

    const std::string stereo_uri = camera_param_uri + "/" + "right.yaml";
    std::string name = "stereo";
    camera_info_manager::CameraInfoManager stereo_cam_manager(ros::NodeHandle{pnh, name}, name, stereo_uri);
    auto stereo_camera_info = stereo_cam_manager.getCameraInfo();
    stereoCamInfoPub.publish(stereo_camera_info);
    // Till here------------------------------------->


    // if (imageDataQueues.size() != imgPubList.size()) {
    //     throw std::runtime_error("Not enough publishers were created for the number of streams from the device");
    // }

    // dai::rosBridge::ImageConverter converter(deviceName + "_left_camera_optical_frame");
    // dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> leftPublish(imageDataQueues[0],
    //                                                                                  pnh, 
    //                                                                                  std::string("left/image"),
    //                                                                                  std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
    //                                                                                  &converter, 
    //                                                                                  std::placeholders::_1, 
    //                                                                                  std::placeholders::_2) , 
    //                                                                                  30);

    // // bridgePublish.startPublisherThread();
    // leftPublish.addPubisherCallback();

    // dai::rosBridge::ImageConverter rightconverter(deviceName + "_right_camera_optical_frame");
    // dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rightPublish(imageDataQueues[1],
    //                                                                                  pnh, 
    //                                                                                  std::string("right/image"),
    //                                                                                  std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
    //                                                                                  &rightconverter, 
    //                                                                                  std::placeholders::_1, 
    //                                                                                  std::placeholders::_2) , 
    //                                                                                  30);

    // rightPublish.addPubisherCallback();

    dai::rosBridge::ImageConverter depthConverter(deviceName + "_right_camera_optical_frame");
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(imageDataQueues[0],
                                                                                     pnh, 
                                                                                     std::string("stereo/depth"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &depthConverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30);


    dai::rosBridge::ImageConverter rgbConverter(deviceName + "_rgb_camera_optical_frame");
    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(imageDataQueues[1],
                                                                                     pnh, 
                                                                                     std::string("color/image"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &rgbConverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30);

    depthPublish.addPubisherCallback();
    rgbPublish.startPublisherThread();

    // We can add the rectified frames also similar to these publishers. 
    // Left them out so that users can play with it by adding and removing

    ros::spin();

    return 0;
}

