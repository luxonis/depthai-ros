#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <unordered_map>

#include "depthai/depthai.hpp"

#ifdef IS_ROS2
    #include "rclcpp/rclcpp.hpp"
    #include "sensor_msgs/image_encodings.hpp"
    #include "stereo_msgs/msg/disparity_image.hpp"

#else
    #include <ros/ros.h>

    #include <boost/make_shared.hpp>

    #include "sensor_msgs/image_encodings.h"
    #include "stereo_msgs/DisparityImage.h"

#endif

namespace dai {

#ifdef IS_ROS2
namespace DisparityMsgs = stereo_msgs::msg;
using DisparityImagePtr = DisparityMsgs::DisparityImage::SharedPtr;
#else
namespace DisparityMsgs = stereo_msgs;
using DisparityImagePtr = DisparityMsgs::DisparityImage::Ptr;
#endif
namespace ros {
using TimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>;

class DisparityConverter {
   public:
    DisparityConverter(const std::string frameName, float focalLength, float baseline = 7.5, float minDepth = 80, float maxDepth = 1100);

    void toRosMsg(std::shared_ptr<dai::ImgFrame> inData, DisparityMsgs::DisparityImage& outImageMsg);
    DisparityImagePtr toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData);

    // void toDaiMsg(const DisparityMsgs::DisparityImage& inMsg, dai::ImgFrame& outData);

   private:
    const std::string _frameName = "";
    const float _focalLength = 882.2, _baseline = 7.5, _minDepth = 80, _maxDepth;
};

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
