
#include "ros/ros.h"

#include <iostream>
#include <cstdio>
// #include "utility.hpp"
#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include <depthai_examples/rgb_pipeline.hpp>
#include <depthai_examples/daiUtility.hpp>
// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"


int main(int argc, char** argv){


    ros::init(argc, argv, "rgb_node");
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

    
    RgbCameraPipelineExample rgbPipeline;
    rgbPipeline.initDepthaiDev();
    std::vector<std::shared_ptr<dai::DataOutputQueue>> imageDataQueues = rgbPipeline.getExposedImageStreams();
    
    std::vector<ros::Publisher> imgPubList;
    std::vector<std::string> frameNames;

    for (auto op_que : imageDataQueues){
        if (op_que->getName().find("video") != std::string::npos){
                imgPubList.push_back(pnh.advertise<sensor_msgs::Image>("color/image", 30));
                frameNames.push_back(deviceName + "_rgb_camera_optical_frame");
        }
    }

    bool latched_cam_info = true;
    ros::Publisher colorCamInfoPub   = pnh.advertise<sensor_msgs::CameraInfo>("color/camera_info", 30, latched_cam_info);    

    // this part would be removed once we have calibration-api
    const std::string color_uri = camera_param_uri +"/" + "color.yaml";
    std::string name = "color";
    camera_info_manager::CameraInfoManager color_cam_manager(ros::NodeHandle{pnh, name}, name, color_uri);
    auto color_camera_info = color_cam_manager.getCameraInfo();
    colorCamInfoPub.publish(color_camera_info);
    ROS_INFO("Publishing camera info of color camera........");

    // Till here-------------------------------------> //

    if (imageDataQueues.size() != imgPubList.size()) {
        throw std::runtime_error("Not enough publishers were created for the number of streams from the device");
    }

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

