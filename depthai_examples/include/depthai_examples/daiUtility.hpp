#pragma once

#include <sstream>
#include <iostream>
#include <unordered_map>

#include "sensor_msgs/Image.h"
#include "depthai/depthai.hpp"
#include <vision_msgs/Detection2DArray.h>
#include <depthai_ros_msgs/DetectionDaiArray.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// namespace std {
//     template< class T, class U >
//     inline constexpr bool is_same_v = is_same<T, U>::value;
// }
namespace dai {
using timePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>;

std::unordered_map<dai::RawImgFrame::Type, std::string> encoding_enum_map({
        {dai::RawImgFrame::Type::YUV422i        , "yuv422"               },
        {dai::RawImgFrame::Type::RGBA8888       , "rgba8"                },
        {dai::RawImgFrame::Type::RGB888i        , "rgb8"                 },
        {dai::RawImgFrame::Type::BGR888i        , "bgr8"                 },
        {dai::RawImgFrame::Type::BGR888p        , "planar_3_1_bgr8"      },
        {dai::RawImgFrame::Type::GRAY8          , "8UC1"                 },
        {dai::RawImgFrame::Type::RAW8           , "mono8"                },
        {dai::RawImgFrame::Type::RAW16          , "mono16"               },
        {dai::RawImgFrame::Type::NV12           , "NV12"                 } 
        // {dai::RawImgFrame::Type::NV12           : "CV_bridge" },
    });



template<typename Msg, 
std::enable_if_t<std::is_same<Msg, depthai_ros_msgs::DetectionDai>::value, bool> = true>
void detectionMsgHelper(Msg& rosDetection, dai::ImgDetection& daiDetection){
    rosDetection.position.x = daiDetection.xdepth;
    rosDetection.position.y = daiDetection.ydepth;
    rosDetection.position.z = daiDetection.zdepth;
}

template<typename Msg, 
std::enable_if_t<not std::is_same<Msg, depthai_ros_msgs::DetectionDai>::value, bool> = true>
void detectionMsgHelper(Msg& rosDetection, dai::ImgDetection& daiDetection){
}


void planarToInterleaved(const std::vector<uint8_t>& srcData, std::vector<uint8_t>& destData, int w, int h , int numPlanes, int bpp){
    
    cv::Mat frame;

    if(numPlanes == 3){
        
        // optimization (cache)
        for(int i = 0; i < w*h; i++) {
            uint8_t b = srcData.data()[i + w*h * 0];
            destData[i*3+0] = b;
        }
        for(int i = 0; i < w*h; i++) {                
            uint8_t g = srcData.data()[i + w*h * 1];    
            destData[i*3+1] = g;
        }
        for(int i = 0; i < w*h; i++) {
            uint8_t r = srcData.data()[i + w*h * 2];
            destData[i*3+2] = r;
        }
                    
    } 
    else{
         std::runtime_error("If you encounter the scenario where you need this please create an issue on github");
    }
    return;
}



void rosImageBridge(std::shared_ptr<dai::ImgFrame> inData, std::string frameName, sensor_msgs::Image& outImageMsg){

    
    if (encoding_enum_map.find(inData->getType()) == encoding_enum_map.end())
        throw std::runtime_error("Encoding value node found ");

    //TODO(sachin) : get this time stamp convertion working
    auto tstamp = inData->getTimestamp();
    // std::cout << "time ->  " << encoding_enum_map[inData->getType()] << "  " << (int)inData->getType() << std::endl;
    int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(tstamp.time_since_epoch()).count();
    int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(tstamp.time_since_epoch()).count() % 1000000000UL;
    // ros::Time rTime(sec, nsec);

    // setting the header
    std_msgs::Header imgHeader; 
    imgHeader.seq      = inData->getSequenceNum();
    imgHeader.stamp    = ros::Time(sec, nsec);;
    imgHeader.frame_id = frameName;

    if(encoding_enum_map[inData->getType()] == "NV12"){
        //TODO(sachin): Replace cv_mat->cvbridge with handling raw convertions
        // ------------------------------------------------------------------------------------ //
        cv::Mat nv_frame(inData->getHeight() * 3 / 2, inData->getWidth(), CV_8UC1, inData->getData().data());
        cv::Mat rgb(inData->getHeight(), inData->getWidth(), CV_8UC3);
        cv::cvtColor(nv_frame, rgb, cv::COLOR_YUV2BGR_NV12);
        cv_bridge::CvImage cvRgbImg(imgHeader, sensor_msgs::image_encodings::BGR8, rgb);
        cvRgbImg.toImageMsg(outImageMsg);
    }
    else if(encoding_enum_map[inData->getType()].find("planar") != std::string::npos){
        
        std::istringstream f(encoding_enum_map[inData->getType()]);
        std::vector<std::string> encoding_info;    
        std::string s;
        
        while (getline(f, s, '_')) 
            encoding_info.push_back(s);
        outImageMsg.header        = imgHeader;
        outImageMsg.encoding      = encoding_info[3];
        outImageMsg.height       = inData->getHeight();
        outImageMsg.width        = inData->getWidth();
        outImageMsg.step         = inData->getData().size() / inData->getHeight();
        outImageMsg.is_bigendian = true;
        size_t size = inData->getData().size();
        outImageMsg.data.resize(size);
        planarToInterleaved(inData->getData(), outImageMsg.data, outImageMsg.width, outImageMsg.height , std::stoi(encoding_info[1]), std::stoi(encoding_info[2]));
    }
    else{
        // copying the data to ros msg
        outImageMsg.header       = imgHeader;
        outImageMsg.encoding     = encoding_enum_map[inData->getType()];
        outImageMsg.height       = inData->getHeight();
        outImageMsg.width        = inData->getWidth();
        outImageMsg.step         = inData->getData().size() / inData->getHeight();
        // std::cout << inData->getData().size() << "..." << inData->getHeight() << std::endl;
        if (outImageMsg.encoding == "mono16")
            outImageMsg.is_bigendian = false;
        else
            outImageMsg.is_bigendian = true;

        size_t size = inData->getData().size();
        outImageMsg.data.resize(size);
        unsigned char* imageMsgDataPtr = reinterpret_cast<unsigned char*>(&outImageMsg.data[0]);
        unsigned char* daiImgData = reinterpret_cast<unsigned char*>(inData->getData().data());
        // TODO(Sachin): Try using assign since it is a vector img->data.assign(packet.data->cbegin(), packet.data->cend());
        memcpy(imageMsgDataPtr, daiImgData, size);
    }
    return;
}
// vision_msgs::Detection2DArray/
template <typename T>
void rosDetectionBridge(std::shared_ptr<dai::ImgDetections> inNetData, timePoint tStamp, unsigned int sequenceNum , std::string frameName, T& opDetectionMsg, int w, int h, std::string trackingId = "", bool isTracking = false, bool normalized = false){

    if(inNetData->detections.size() == 0)
        throw std::runtime_error("Make sure to send the detections with size greater than 0");

    // vision_msgs::Detection2DArrayPtr detectionPtrMsg;
    // time conversion to ros
    int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(tStamp.time_since_epoch()).count();
    int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(tStamp.time_since_epoch()).count() % 1000000000UL;
    
    // setting the header
    std_msgs::Header imgHeader; 
    imgHeader.seq      = sequenceNum;
    imgHeader.stamp    = ros::Time(sec, nsec);;
    imgHeader.frame_id = frameName;

    // setting the header
    opDetectionMsg.header.seq = sequenceNum;
    opDetectionMsg.detections.resize(inNetData->detections.size());

    // TODO(Sachin): check if this works fine for normalized detection publishing
    for(int i = 0; i < inNetData->detections.size(); ++i){
        int xMin, yMin, xMax, yMax;
        if (normalized){
            xMin = inNetData->detections[i].xmin; 
            yMin = inNetData->detections[i].ymin;
            xMax = inNetData->detections[i].xmax;
            yMax = inNetData->detections[i].ymax;
        }
        else{
            xMin = inNetData->detections[i].xmin * w;
            yMin = inNetData->detections[i].ymin * h;
            xMax = inNetData->detections[i].xmax * w;
            yMax = inNetData->detections[i].ymax * h;
        }

        float xSize = xMax - xMin;
        float ySize = yMax - yMin;
        float xCenter = xMin + xSize / 2;
        float yCenter = yMin + ySize / 2;

        opDetectionMsg.detections[i].results.resize(1);
        opDetectionMsg.detections[i].results[0].id    = std::to_string(inNetData->detections[i].label);
        opDetectionMsg.detections[i].results[0].score = inNetData->detections[i].confidence;

        opDetectionMsg.detections[i].bbox.center.x = xCenter;
        opDetectionMsg.detections[i].bbox.center.y = yCenter;
        opDetectionMsg.detections[i].bbox.size_x   = xSize;
        opDetectionMsg.detections[i].bbox.size_y   = ySize;
        opDetectionMsg.detections[i].is_tracking   = isTracking;
        
        if(isTracking){
            opDetectionMsg.detections[i].tracking_id = trackingId;
        }
        detectionMsgHelper(opDetectionMsg.detections[i], inNetData->detections[i]);
    }
}

};