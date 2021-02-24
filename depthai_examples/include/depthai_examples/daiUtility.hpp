#pragma once

#include <unordered_map>
#include "sensor_msgs/Image.h"
#include "depthai/depthai.hpp"
#include <vision_msgs/Detection2DArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace dai {
using timePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>;

std::unordered_map<dai::RawImgFrame::Type, std::string> encoding_enum_map({
        {dai::RawImgFrame::Type::YUV422i        , "yuv422"    },
        {dai::RawImgFrame::Type::RGBA8888       , "rgba8"     },
        {dai::RawImgFrame::Type::RGB888i        , "rgb8"      },
        {dai::RawImgFrame::Type::BGR888i        , "bgr8"      },
        {dai::RawImgFrame::Type::GRAY8          , "8UC1"      },
        {dai::RawImgFrame::Type::RAW8           , "mono8"     },
        {dai::RawImgFrame::Type::RAW16          , "mono16"    },
        {dai::RawImgFrame::Type::NV12           , "NV12"      } 
        // {dai::RawImgFrame::Type::NV12           : "CV_bridge" },
    });


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
    else{
        // copying the data to ros msg
        outImageMsg.header            = imgHeader;
        outImageMsg.encoding          = encoding_enum_map[inData->getType()];
        outImageMsg.height            = inData->getHeight();
        outImageMsg.width             = inData->getWidth();
        outImageMsg.step              = inData->getData().size() / inData->getHeight();
        // std::cout << inData->getData().size() << "..." << inData->getHeight() << std::endl;
        if (outImageMsg.encoding == "mono16")
            outImageMsg.is_bigendian = false;
        else
            outImageMsg.is_bigendian = true;

        size_t size = outImageMsg.step * outImageMsg.height;
        outImageMsg.data.resize(size);
        unsigned char* imageMsgDataPtr = reinterpret_cast<unsigned char*>(&outImageMsg.data[0]);
        unsigned char* daiImgData = reinterpret_cast<unsigned char*>(inData->getData().data());
        // TODO(Sachin): Try using assign since it is a vector img->data.assign(packet.data->cbegin(), packet.data->cend());
        memcpy(imageMsgDataPtr, daiImgData, size);
    }

    return;
}


void rosDetectionBridge(std::shared_ptr<dai::ImgDetections> inNetData, timePoint tStamp, unsigned int sequenceNum , std::string frameName, std::string trackingId, vision_msgs::Detection2DArray& opDetectionMsg, bool isTracking = false){

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
    opDetectionMsg.header.seq= sequenceNum;
    opDetectionMsg.detections.resize(inNetData->detections.size());

    for(int i = 0; i < inNetData->detections.size(); ++i){
        vision_msgs::Detection2D &local_object =  opDetectionMsg.detections[i];

        int xMin = inNetData->detections[i].xmin; 
        int yMin = inNetData->detections[i].ymin;
        int xMax = inNetData->detections[i].xmax;
        int yMax = inNetData->detections[i].ymax;

        float xSize = xMax - xMin;
        float ySize = yMax - yMin;
        float xCenter = xMin + xSize / 2;
        float yCenter = yMin + ySize / 2;

        local_object.results.resize(1);
        local_object.results[0].id    = std::to_string(inNetData->detections[i].label);
        local_object.results[0].score = inNetData->detections[i].confidence;

        local_object.bbox.center.x = xCenter;
        local_object.bbox.center.y = yCenter;
        local_object.bbox.size_x   = xSize;
        local_object.bbox.size_y   = ySize;
        local_object.is_tracking = isTracking;
        
        if(isTracking){
            local_object.tracking_id = trackingId;
        }
    }
}



};