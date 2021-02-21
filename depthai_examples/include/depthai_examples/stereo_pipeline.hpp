
#pragma once

#include <iostream> // do I need this ?
#include "depthai/depthai.hpp"


class StereoExampe{
    public:
    StereoExampe() = default;
    ~StereoExampe() = default;

    void initDepthaiDev();

    std::vector<std::shared_ptr<dai::DataOutputQueue>> getExposedImageStreams();

    private:
    std::vector<std::shared_ptr<dai::DataOutputQueue>> _opImageStreams;
    std::unique_ptr<dai::Device> _dev;
    dai::Pipeline _p;

};


// void stereo_node(){

//     using namespace std;
//     // ros::init(argc, argv, "stereo_node");
//     ros::NodeHandle pnh("~");
//     bool outputDepth;
//     bool outputRectified;
//     bool lrcheck;
//     bool extended;
//     bool subpixel;
//     bool withDepth = true;
    
//     dai::Pipeline p;

//     auto monoLeft    = p.create<dai::node::MonoCamera>();
//     auto monoRight   = p.create<dai::node::MonoCamera>();
//     auto xoutLeft    = p.create<dai::node::XLinkOut>();
//     auto xoutRight   = p.create<dai::node::XLinkOut>();
//     auto stereo      = withDepth ? p.create<dai::node::StereoDepth>() : nullptr;
//     auto xoutDisp    = p.create<dai::node::XLinkOut>();
//     auto xoutDepth   = p.create<dai::node::XLinkOut>();
//     auto xoutRectifL = p.create<dai::node::XLinkOut>();
//     auto xoutRectifR = p.create<dai::node::XLinkOut>();

//     // XLinkOut
//     xoutLeft->setStreamName("left");
//     xoutRight->setStreamName("right");
//     if (withDepth) {
//         xoutDisp->setStreamName("disparity");
//         xoutDepth->setStreamName("depth");
//         xoutRectifL->setStreamName("rectified_left");
//         xoutRectifR->setStreamName("rectified_right");
//     }

//     // MonoCamera
//     monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
//     monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
//     //monoLeft->setFps(5.0);
//     monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
//     monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
//     //monoRight->setFps(5.0);



//     int maxDisp = 96;
//     if (extended) maxDisp *= 2;
//     if (subpixel) maxDisp *= 32; // 5 bits fractional disparity

//     if (withDepth) {
//         // StereoDepth
//         stereo->setOutputDepth(outputDepth);
//         stereo->setOutputRectified(outputRectified);
//         stereo->setConfidenceThreshold(200);
//         stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
//         //stereo->loadCalibrationFile("../../../../depthai/resources/depthai.calib");
//         //stereo->setInputResolution(1280, 720);
//         // TODO: median filtering is disabled on device with (lrcheck || extended || subpixel)
//         //stereo->setMedianFilter(dai::StereoDepthProperties::MedianFilter::MEDIAN_OFF);
//         stereo->setLeftRightCheck(lrcheck);
//         stereo->setExtendedDisparity(extended);
//         stereo->setSubpixel(subpixel);

//         // Link plugins CAM -> STEREO -> XLINK
//         monoLeft->out.link(stereo->left);
//         monoRight->out.link(stereo->right);

//         stereo->syncedLeft.link(xoutLeft->input);
//         stereo->syncedRight.link(xoutRight->input);
//         if(outputRectified)
//         {
//             stereo->rectifiedLeft.link(xoutRectifL->input);
//             stereo->rectifiedRight.link(xoutRectifR->input);
//         }
//         stereo->disparity.link(xoutDisp->input);
//         stereo->depth.link(xoutDepth->input);

//     } else {
//         // Link plugins CAM -> XLINK
//         monoLeft->out.link(xoutLeft->input);
//         monoRight->out.link(xoutRight->input);
//     }

//     // CONNECT TO DEVICE
//     dai::Device d(p);
//     d.startPipeline();

//     auto leftQueue = d.getOutputQueue("left", 30, false);
//     auto rightQueue = d.getOutputQueue("right", 30, false);

//     auto dispQueue = withDepth ? d.getOutputQueue("disparity", 30, false) : nullptr;
//     auto depthQueue = withDepth ? d.getOutputQueue("depth", 30, false) : nullptr;
//     auto rectifLeftQueue = withDepth ? d.getOutputQueue("rectified_left", 30, false) : nullptr;
//     auto rectifRightQueue = withDepth ? d.getOutputQueue("rectified_right", 30, false) : nullptr;
    
//     // ros publisher init
//     bool latched_cam_info = true;
//     ros::Publisher leftPub          = pnh.advertise<sensor_msgs::Image>("left/image", 30);    
//     ros::Publisher leftCamInfoPub   = pnh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 30, latched_cam_info);    
    
//     ros::Publisher rightPub         = pnh.advertise<sensor_msgs::Image>("right/image", 30);    
//     ros::Publisher rightCamInfoPub  = pnh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 30, latched_cam_info);    
    
//     ros::Publisher dispPub          = pnh.advertise<sensor_msgs::Image>("stereo/disparity", 30);    
//     ros::Publisher depthPub         = pnh.advertise<sensor_msgs::Image>("stereo/depth_raw", 30);    
//     ros::Publisher stereoCamInfoPub = pnh.advertise<sensor_msgs::CameraInfo>("stereo/camera_info", 30, latched_cam_info);    
    
//     ros::Publisher rectiLeftPub     = pnh.advertise<sensor_msgs::Image>("left/image_rect", 30);    
//     ros::Publisher rectiRightPub    = pnh.advertise<sensor_msgs::Image>("right/image_rect", 30);    

//     const std::string left_uri = camera_param_uri +"/" + "left.yaml";
//     std::string name = "left";
//     camera_info_manager::CameraInfoManager left_cam_manager(ros::NodeHandle{pnh, name}, name, left_uri);
//     auto left_camera_info = left_cam_manager.getCameraInfo();
//     leftCamInfoPub.publish(left_camera_info);
//     ROS_INFO("Publishing camera onfo of left camera........");

//     const std::string right_uri = camera_param_uri + "/" + "right.yaml";
//     name = "right";
//     camera_info_manager::CameraInfoManager right_cam_manager(ros::NodeHandle{pnh, name}, name, right_uri);
//     auto right_camera_info = right_cam_manager.getCameraInfo();
//     rightCamInfoPub.publish(right_camera_info);
    
// // camera_info_manager::CameraInfoManager left_cam_manager  =std::make_unique<camera_info_manager::CameraInfoManager>(ros::NodeHandle{nh, name}, name, uri);
    
//     while (ros::ok()) {
//         auto left = leftQueue->get<dai::ImgFrame>();
//         std::string frameName = deviceName + "_left_camera_optical_frame";      
//         leftPub.publish(dai::rosImageBridge(left, frameName););

//         auto right = rightQueue->get<dai::ImgFrame>();
//         frameName = deviceName + "_right_camera_optical_frame"; 
//         rightPub.publish(dai::rosImageBridge(right, frameName));
        
//         if (withDepth) {
//             // Note: in some configurations (if depth is enabled), disparity may output garbage data
//             auto disparity = dispQueue->get<dai::ImgFrame>();
            
//             frameName = deviceName + "_right_camera_optical_frame"; 
//             dispPub.publish(dai::rosImageBridge(disparity, frameName));
          

//             if (outputDepth) {
//                 auto depth = depthQueue->get<dai::ImgFrame>();
//                 frameName = deviceName + "_right_camera_optical_frame"; 
//                 depthPub.publish(dai::rosImageBridge(depth, frameName));
//             }

//             if (outputRectified) {
//                 auto rectifL = rectifLeftQueue->get<dai::ImgFrame>();
//                 // cv::imshow("rectified_left", cv::Mat(rectifL->getHeight(), rectifL->getWidth(),
//                 //         CV_8UC1, rectifL->getData().data()));
//                 frameName = deviceName + "_left_camera_optical_frame"; 
//                 rectiLeftPub.publish(dai::rosImageBridge(rectifL, rectiLeftPub, frameName));

//                 auto rectifR = rectifRightQueue->get<dai::ImgFrame>();
//                 frameName = deviceName + "_right_camera_optical_frame"; 
//                 rectiRightPub.publish(dai::rosImageBridge(rectifR, rectiRightPub, frameName));
//             }
//         }
//         ros::spinOnce();
//     }
//     return;
// }

// THoughts. 

// Create a class and pipeline in a function(constructor) followed by creating pipeline and creating threads for nodes that needs to be run on the thread in it's own loop.
// create an another function which needs to be called in while loop on the main thread 
// and the final streams that needs to be published will be returned by an another function from private variable may be.
// Then main function in same file or mostly different file will initialize the class and create a while loop and call the function that needs to be run on loop in the main function. and write the code to do what they want with the external output. 
// which in my case will be published as ros publisers.   
