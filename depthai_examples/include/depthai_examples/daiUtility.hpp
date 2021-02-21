#pragma once

#include <unordered_map>
#include "sensor_msgs/Image.h"
#include "depthai/depthai.hpp"

namespace dai {


 std::unordered_map<dai::RawImgFrame::Type, std::string> encoding_enum_map({
        {dai::RawImgFrame::Type::YUV422i        , "yuv422"    },
        {dai::RawImgFrame::Type::RGBA8888       , "rgba8"     },
        {dai::RawImgFrame::Type::RGB888i        , "rgb8"      },
        {dai::RawImgFrame::Type::BGR888i        , "bgr8"      },
        {dai::RawImgFrame::Type::GRAY8          , "8UC1"      },
        {dai::RawImgFrame::Type::RAW8           , "mono8"     },
        {dai::RawImgFrame::Type::RAW16          , "mono16"    }
        // {dai::RawImgFrame::Type::NV12           : "CV_bridge" },
    });


sensor_msgs::Image rosImageBridge(std::shared_ptr<dai::ImgFrame> inData, std::string frameName){

    sensor_msgs::Image imageMsg;
    
    if (encoding_enum_map.find(inData->getType()) == encoding_enum_map.end())
        throw std::runtime_error("Encoding value node found ");

    //TODO(sachin) : get this time stamp convertion working
    auto tstamp = inData->getTimestamp();
    std::cout << "time ->  " << encoding_enum_map[inData->getType()] << "  " << (int)inData->getType() << std::endl;
    int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(tstamp.time_since_epoch()).count();
    int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(tstamp.time_since_epoch()).count() % 1000000000UL;
    ros::Time rTime(sec, nsec);

    // setting the header
    imageMsg.header.seq      = inData->getSequenceNum();
    imageMsg.header.stamp = rTime;
    imageMsg.header.frame_id = frameName;
    // copying the data to ros msg
    imageMsg.encoding          = encoding_enum_map[inData->getType()];
    imageMsg.height            = inData->getHeight();
    imageMsg.width             = inData->getWidth();
    imageMsg.step              = inData->getData().size() / inData->getHeight();
    std::cout << inData->getData().size() << "..." << inData->getHeight() << std::endl;
    if (imageMsg.encoding == "mono16")
        imageMsg.is_bigendian = false;
    else
        imageMsg.is_bigendian = true;

    size_t size = imageMsg.step * imageMsg.height;
    imageMsg.data.resize(size);
    unsigned char* imageMsgDataPtr = reinterpret_cast<unsigned char*>(&imageMsg.data[0]);
    unsigned char* daiImgData = reinterpret_cast<unsigned char*>(inData->getData().data());
    
    memcpy(imageMsgDataPtr, daiImgData, size);
    return imageMsg;
}
};