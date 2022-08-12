#pragma once

#include <depthai_bridge/depthaiUtility.hpp>
#include <deque>

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

namespace ros {

#ifdef IS_ROS2
namespace DisparityMsgs = stereo_msgs::msg;
namespace ImageMsgs = sensor_msgs::msg;
using ImagePtr = ImageMsgs::Image::SharedPtr;
using DisparityImagePtr = DisparityMsgs::DisparityImage::SharedPtr;
#else
namespace DisparityMsgs = stereo_msgs;
namespace ImageMsgs = sensor_msgs;
using ImagePtr = ImageMsgs::ImagePtr;
using DisparityImagePtr = DisparityMsgs::DisparityImage::Ptr;
#endif
using TimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>;

class DisparityConverter {
   public:
    DisparityConverter(const std::string frameName, float focalLength, float baseline = 7.5, float minDepth = 80, float maxDepth = 1100);

    void toRosMsg(std::shared_ptr<dai::ImgFrame> inData, std::deque<DisparityMsgs::DisparityImage>& outImageMsg);
    DisparityImagePtr toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData);

    // void toDaiMsg(const DisparityMsgs::DisparityImage& inMsg, dai::ImgFrame& outData);

   private:
    const std::string _frameName = "";
    const float _focalLength = 882.2, _baseline = 7.5, _minDepth = 80, _maxDepth;
    std::chrono::time_point<std::chrono::steady_clock> _steadyBaseTime;
#ifdef IS_ROS2
    rclcpp::Time _rosBaseTime;
#else
    ::ros::Time _rosBaseTime;
#endif
};

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
