
#include "ros/ros.h"

#include <iostream>
#include <cstdio>
// #include "utility.hpp"
#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include <depthai_examples/stereo_pipeline.hpp>
#include <depthai_examples/daiUtility.hpp>
// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"


int main(int argc, char** argv){


    ros::init(argc, argv, "stereo_node");
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

    
    StereoExampe stero_pipeline;
    stero_pipeline.initDepthaiDev();
    std::vector<std::shared_ptr<dai::DataOutputQueue>> imageDataQueues = stero_pipeline.getExposedImageStreams();
    
    std::vector<ros::Publisher> imgPubList;
    std::vector<std::string> frameNames;

    for (auto op_que : imageDataQueues){
        if (op_que->getName().find("rect") != std::string::npos){
            if (op_que->getName().find("left") != std::string::npos)
            {    
                imgPubList.push_back(pnh.advertise<sensor_msgs::Image>("left/image_rect", 30));
                frameNames.push_back(deviceName + "_left_camera_optical_frame");
            }
            if (op_que->getName().find("right") != std::string::npos)
            {
                imgPubList.push_back(pnh.advertise<sensor_msgs::Image>("right/image_rect", 30));
                frameNames.push_back(deviceName + "_right_camera_optical_frame");
            }
        }
        else{
            if (op_que->getName().find("left") != std::string::npos){
                imgPubList.push_back(pnh.advertise<sensor_msgs::Image>("left/image", 30));
                frameNames.push_back(deviceName + "_left_camera_optical_frame");
            }
            else if (op_que->getName().find("right") != std::string::npos){
                imgPubList.push_back(pnh.advertise<sensor_msgs::Image>("right/image", 30));
                frameNames.push_back(deviceName + "_right_camera_optical_frame");
            }
            else if (op_que->getName().find("depth") != std::string::npos){
                imgPubList.push_back(pnh.advertise<sensor_msgs::Image>("stereo/depth", 30));
                frameNames.push_back(deviceName + "_right_camera_optical_frame");
            }
            else if (op_que->getName().find("disparity") != std::string::npos){
                imgPubList.push_back(pnh.advertise<sensor_msgs::Image>("stereo/disparity", 30));
                frameNames.push_back(deviceName + "_right_camera_optical_frame");
            }
        }
    }

    bool latched_cam_info = true;
    ros::Publisher leftCamInfoPub   = pnh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 30, latched_cam_info);    
    ros::Publisher rightCamInfoPub  = pnh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 30, latched_cam_info);    
    ros::Publisher stereoCamInfoPub = pnh.advertise<sensor_msgs::CameraInfo>("stereo/camera_info", 30, latched_cam_info);    

    // this part would be removed once we have calibration-api
    const std::string left_uri = camera_param_uri +"/" + "left.yaml";
    std::string name = "left";
    camera_info_manager::CameraInfoManager left_cam_manager(ros::NodeHandle{pnh, name}, name, left_uri);
    auto left_camera_info = left_cam_manager.getCameraInfo();
    leftCamInfoPub.publish(left_camera_info);
    ROS_INFO("Publishing camera onfo of left camera........");

    const std::string right_uri = camera_param_uri + "/" + "right.yaml";
    name = "right";
    camera_info_manager::CameraInfoManager right_cam_manager(ros::NodeHandle{pnh, name}, name, right_uri);
    auto right_camera_info = right_cam_manager.getCameraInfo();
    rightCamInfoPub.publish(right_camera_info);

    const std::string stereo_uri = camera_param_uri + "/" + "right.yaml";
    name = "stereo";
    camera_info_manager::CameraInfoManager stereo_cam_manager(ros::NodeHandle{pnh, name}, name, stereo_uri);
    auto stereo_camera_info = stereo_cam_manager.getCameraInfo();
    stereoCamInfoPub.publish(stereo_camera_info);
    // Till here------------------------------------->


    if (imageDataQueues.size() != imgPubList.size()) {
        throw std::runtime_error("Not enough publishers were created for the number of streams from the device");
    }

    
    // std::cout << "Waiting for " << imageDataQueues.size() << std::endl;
    
    while(ros::ok()){
        for(int i = 0; i < imageDataQueues.size(); ++i){
            if(imgPubList[i].getNumSubscribers() == 0) continue;
            auto imgData = imageDataQueues[i]->get<dai::ImgFrame>();
            // std::cout << "id num ->" << i << imageDataQueues[i]->getName() << std::endl;
            sensor_msgs::Image imageMsg;
            dai::rosImageBridge(imgData, frameNames[i], imageMsg);
            imgPubList[i].publish(imageMsg);
        }
        ros::spinOnce();
    }


    return 0;
}

