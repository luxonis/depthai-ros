

#include <iostream>
#include <cstdio>
#include <camera_info_manager.h>
#include "utility.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

const std::unordered_map<int, std::string> encoding_enum_map({
        {dai::RawImgFrame::Type::YUV422i        : "yuv422"    },
        {dai::RawImgFrame::Type::RGBA8888       : "rgba8"     },
        {dai::RawImgFrame::Type::RGB888i        : "rgb8"      },
        {dai::RawImgFrame::Type::BGR888i        : "bgr8"      },
        {dai::RawImgFrame::Type::GRAY8          : "8UC1"      },
        {dai::RawImgFrame::Type::RAW8           : "8UC1"      },
        {dai::RawImgFrame::Type::RAW16          : "16UC1"     }
        // {dai::RawImgFrame::Type::NV12           : "CV_bridge" },
    });


void converter(std::shared_ptr<dai::ImgFrame> inData, ros::Publisher &pub, std::string frameName){

    sensor_msgs::ImagePtr imageMsg;
    if (encoding_enum_map.find(inData->getType()) == encoding_enum_enum.end())
        throw std::runtime_error("Encoding value node found %d", inData->getType());

    imageMsg->encoding        = encoding_enum_map[inData->getType()];
    imageMsg->header.seq      = inData->getSequenceNum();
    imageMsg->header.stamp    = inData->getTimestamp();
    imageMsg->header.frame_id = frameName;

    imageMsg->height = inData->getHeight();
    imageMsg->width  = inData->getWidth();
    imageMsg->step   = sizeof(uint8_t) * inData->getWidth();
    imageMsg->data   = inData->getData().data();

    pub.publish(imageMsg);
}


int main(){

    using namespace std;
    ros::init(argc, argv, "stereo_node");
    ros::NodeHandle pnh("~");
    bool outputDepth;
    bool outputRectified;
    bool lrcheck;
    bool extended;
    bool subpixel;
    std::string cameraName;
    std::string camera_param_uri;
    int bad_params = 0;

    bad_params += !pnh.getParam("outputDepth", outputDepth);
    bad_params += !pnh.getParam("outputRectified", outputRectified);
    bad_params += !pnh.getParam("lrcheck", lrcheck);
    bad_params += !pnh.getParam("extended", extended);
    bad_params += !pnh.getParam("subpixel", subpixel);
    bad_params += !pnh.getParam("cameraName", cameraName);
    bad_params += !pnh.getParam("camera_param_uri", camera_param_uri);

    // TODO - split this example into two separate examples
    if (bad_params > 0)
    {
        throw std::runtime_error("Couldn't find one of the parameters");
    }

    bool withDepth = true;
    
    dai::Pipeline p;

    auto monoLeft  = p.create<dai::node::MonoCamera>();
    auto monoRight = p.create<dai::node::MonoCamera>();
    auto xoutLeft  = p.create<dai::node::XLinkOut>();
    auto xoutRight = p.create<dai::node::XLinkOut>();
    auto stereo    = withDepth ? p.create<dai::node::StereoDepth>() : nullptr;
    auto xoutDisp  = p.create<dai::node::XLinkOut>();
    auto xoutDepth = p.create<dai::node::XLinkOut>();
    auto xoutRectifL = p.create<dai::node::XLinkOut>();
    auto xoutRectifR = p.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    if (withDepth) {
        xoutDisp->setStreamName("disparity");
        xoutDepth->setStreamName("depth");
        xoutRectifL->setStreamName("rectified_left");
        xoutRectifR->setStreamName("rectified_right");
    }

    // MonoCamera
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    //monoLeft->setFps(5.0);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    //monoRight->setFps(5.0);



    int maxDisp = 96;
    if (extended) maxDisp *= 2;
    if (subpixel) maxDisp *= 32; // 5 bits fractional disparity

    if (withDepth) {
        // StereoDepth
        stereo->setOutputDepth(outputDepth);
        stereo->setOutputRectified(outputRectified);
        stereo->setConfidenceThreshold(200);
        stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
        //stereo->loadCalibrationFile("../../../../depthai/resources/depthai.calib");
        //stereo->setInputResolution(1280, 720);
        // TODO: median filtering is disabled on device with (lrcheck || extended || subpixel)
        //stereo->setMedianFilter(dai::StereoDepthProperties::MedianFilter::MEDIAN_OFF);
        stereo->setLeftRightCheck(lrcheck);
        stereo->setExtendedDisparity(extended);
        stereo->setSubpixel(subpixel);

        // Link plugins CAM -> STEREO -> XLINK
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);

        stereo->syncedLeft.link(xoutLeft->input);
        stereo->syncedRight.link(xoutRight->input);
        if(outputRectified)
        {
            stereo->rectifiedLeft.link(xoutRectifL->input);
            stereo->rectifiedRight.link(xoutRectifR->input);
        }
        stereo->disparity.link(xoutDisp->input);
        stereo->depth.link(xoutDepth->input);

    } else {
        // Link plugins CAM -> XLINK
        monoLeft->out.link(xoutLeft->input);
        monoRight->out.link(xoutRight->input);
    }

    // CONNECT TO DEVICE
    dai::Device d(p);
    d.startPipeline();

    auto leftQueue = d.getOutputQueue("left", 30, false);
    auto rightQueue = d.getOutputQueue("right", 30, false);

    auto leftQueue = d.getOutputQueue("left", 8, false);
    auto rightQueue = d.getOutputQueue("right", 8, false);
    auto dispQueue = withDepth ? d.getOutputQueue("disparity", 8, false) : nullptr;
    auto depthQueue = withDepth ? d.getOutputQueue("depth", 8, false) : nullptr;
    auto rectifLeftQueue = withDepth ? d.getOutputQueue("rectified_left", 8, false) : nullptr;
    auto rectifRightQueue = withDepth ? d.getOutputQueue("rectified_right", 8, false) : nullptr;
    
    bool latched_cam_info = true;
    ros::Publisher leftPub          = pnh.advertise<sensor_msgs::Image>("left/image", 30);    
    ros::Publisher leftCamInfoPub   = pnh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 30, latched_cam_info);    
    
    ros::Publisher rightPub         = pnh.advertise<sensor_msgs::Image>("right/image", 30);    
    ros::Publisher rightCamInfoPub  = pnh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 30, latched_cam_info);    
    
    ros::Publisher dispPub          = pnh.advertise<sensor_msgs::Image>("stereo/disparity", 30);    
    ros::Publisher depthPub         = pnh.advertise<sensor_msgs::Image>("stereo/depth_raw", 30);    
    ros::Publisher stereoCamInfoPub = pnh.advertise<sensor_msgs::CameraInfo>("stereo/camera_info", 30, latched_cam_info);    
    
    ros::Publisher rectiLeftPub     = pnh.advertise<sensor_msgs::Image>("right/image_rect", 30);    
    ros::Publisher rectiRightPub    = pnh.advertise<sensor_msgs::Image>("left/image_rect", 30);    

    const auto uri = camera_param_uri + camera_name + "/" + "left.yaml";
    camera_info_manager::CameraInfoManager left_cam_manager  =std::make_unique<camera_info_manager::CameraInfoManager>(ros::NodeHandle{nh, name}, name, uri);
    const auto left_camera_info = std::make_unique<sensor_msgs::CameraInfo>(left_cam_manager->getCameraInfo());
    leftCamInfoPub.publish(left_camera_info);

    const auto uri = camera_param_uri + camera_name + "/" + "right.yaml";
    camera_info_manager::CameraInfoManager right_cam_manager  =std::make_unique<camera_info_manager::CameraInfoManager>(ros::NodeHandle{nh, name}, name, uri);
    const auto right_camera_info = std::make_unique<sensor_msgs::CameraInfo>(right_cam_manager->getCameraInfo());
    rightCamInfoPub.publish(right_camera_info);
    
// camera_info_manager::CameraInfoManager left_cam_manager  =std::make_unique<camera_info_manager::CameraInfoManager>(ros::NodeHandle{nh, name}, name, uri);
    
    while (1) {
        auto left = leftQueue->get<dai::ImgFrame>();
        std::string frameName = cameraName + "_left_camera_optical_frame"; 
        converter(left, leftPub, frameName);

        auto right = rightQueue->get<dai::ImgFrame>();
        std::string frameName = cameraName + "_right_camera_optical_frame"; 
        converter(right, rightPub, frameName);
        
        if (withDepth) {
            // Note: in some configurations (if depth is enabled), disparity may output garbage data
            auto disparity = dispQueue->get<dai::ImgFrame>();
            
            std::string frameName = cameraName + "_right_camera_optical_frame"; 
            converter(disparity, dispPub, frameName);
          

            if (outputDepth) {
                auto depth = depthQueue->get<dai::ImgFrame>();
                std::string frameName = cameraName + "_right_camera_optical_frame"; 
                converter(depth, depthPub, frameName);
            }

            if (outputRectified) {
                auto rectifL = rectifLeftQueue->get<dai::ImgFrame>();
                std::string frameName = cameraName + "_left_camera_optical_frame"; 
                converter(rectifL, rectiLeftPub, frameName);

                auto rectifR = rectifRightQueue->get<dai::ImgFrame>();
                std::string frameName = cameraName + "_right_camera_optical_frame"; 
                converter(rectifL, rectiLeftPub, frameName);
            }
        }

    }
}

