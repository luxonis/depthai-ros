#pragma once

#include <sstream>
#include <iostream>
#include <unordered_map>

#include "sensor_msgs/Image.h"
#include "depthai/depthai.hpp"
#include <opencv2/opencv.hpp>

namespace dai::rosBridge {
using TimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>;

class ImageConverter{

    public:
    // ImageConverter() = default;
    ImageConverter(const std::string frameName, bool interleaved);
    ImageConverter(bool interleaved);
    
    void toRosMsg(std::shared_ptr<dai::ImgFrame> inData, sensor_msgs::Image& outImageMsg);
    sensor_msgs::ImagePtr toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData);

    void toDaiMsg(sensor_msgs::Image& inMsg, std::shared_ptr<dai::ImgFrame> outData);

    /** TODO(sachin): Add support for ros msg to cv mat since we have some 
     *  encodings which cv supports but ros doesn't    
    **/
    void rosMsgtoCvMat(sensor_msgs::Image& inMsg, cv::Mat dest);


     private:
        static std::unordered_map<dai::RawImgFrame::Type, std::string> encodingEnumMap;
        static std::unordered_map<dai::RawImgFrame::Type, std::string> planarEncodingEnumMap;

    // std::unordered_map<dai::RawImgFrame::Type, std::string> encodingEnumMap({
    //     {dai::RawImgFrame::Type::YUV422i        , "yuv422"               },
    //     {dai::RawImgFrame::Type::RGBA8888       , "rgba8"                },
    //     {dai::RawImgFrame::Type::RGB888i        , "rgb8"                 },
    //     {dai::RawImgFrame::Type::BGR888i        , "bgr8"                 },
    //     {dai::RawImgFrame::Type::GRAY8          , "8UC1"                 },
    //     {dai::RawImgFrame::Type::RAW8           , "mono8"                },
    //     {dai::RawImgFrame::Type::RAW16          , "mono16"               },
    //     {dai::RawImgFrame::Type::NV12           , "NV12"                 } 
    //     // {dai::RawImgFrame::Type::NV12           : "CV_bridge" },
    // });

    // std::unordered_map<dai::RawImgFrame::Type, std::string> planarEncodingEnumMap({
    //             {dai::RawImgFrame::Type::BGR888p  , "3_1_bgr8"      }
    //         });
    

    // dai::RawImgFrame::Type _srcType;
    bool _daiInterleaved;
    // bool c
    const std::string _frameName = "";
    void planarToInterleaved(const std::vector<uint8_t>& srcData, std::vector<uint8_t>& destData, int w, int h , int numPlanes, int bpp);
    void interleavedToPlanar(const std::vector<uint8_t>& srcData, std::vector<uint8_t>& destData, int w, int h , int numPlanes, int bpp);

};

}   // namespace dai::rosBridge