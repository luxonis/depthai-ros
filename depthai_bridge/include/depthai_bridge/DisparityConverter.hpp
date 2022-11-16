#pragma once

#include <ros/ros.h>

#include <boost/make_shared.hpp>
#include <depthai_bridge/depthaiUtility.hpp>
#include <deque>

#include "depthai/depthai.hpp"
#include "sensor_msgs/image_encodings.h"
#include "stereo_msgs/DisparityImage.h"

namespace dai {

namespace ros {

namespace DisparityMsgs = stereo_msgs;
namespace ImageMsgs = sensor_msgs;
using ImagePtr = ImageMsgs::ImagePtr;
using DisparityImagePtr = DisparityMsgs::DisparityImage::Ptr;
using TimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>;

class DisparityConverter {
   public:
    DisparityConverter(const std::string frameName, float focalLength, float baseline = 7.5, float minDepth = 80, float maxDepth = 1100);

    void toRosMsg(std::shared_ptr<dai::ImgFrame> inData, std::deque<DisparityMsgs::DisparityImage>& outImageMsg);
    DisparityImagePtr toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData);

   private:
    const std::string _frameName = "";
    const float _focalLength = 882.2, _baseline = 7.5, _minDepth = 80, _maxDepth;
    std::chrono::time_point<std::chrono::steady_clock> _steadyBaseTime;

    ::ros::Time _rosBaseTime;
};

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
