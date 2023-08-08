#pragma once

#include <deque>
#include <memory>
#include <string>

#include "depthai/pipeline/datatype/Tracklets.hpp"
#include "depthai_ros_msgs/TrackletArray.h"
#include "ros/ros.h"

namespace dai {

namespace ros {
namespace TrackletMessages = depthai_ros_msgs;
using TrackletArrayPtr = TrackletMessages::TrackletArray::Ptr;
class TrackletConverter {
   public:
    TrackletConverter(std::string frameName,
                      int width_srcImgDetection,
                      int height_srcImgDetection,
                      int width_srcImgTracker,
                      int height_srcImgTracker,
                      bool normalized_srcImgDetection = false,
                      bool normalized_srcImgTracker = false,
                      bool getBaseDeviceTimestamp = false);
    void toRosMsg(std::shared_ptr<dai::Tracklets> trackData, std::deque<TrackletMessages::TrackletArray>& trackletsMsg);
    TrackletArrayPtr toRosMsgPtr(std::shared_ptr<dai::Tracklets> trackData);

   private:
    int _width_srcImgDetection, _height_srcImgDetection, _width_srcImgTracker, _height_srcImgTracker;
    const std::string _frameName;
    bool _normalized_srcImgDetection, _normalized_srcImgTracker;
    std::chrono::time_point<std::chrono::steady_clock> _steadyBaseTime;
    ::ros::Time _rosBaseTime;
    bool _getBaseDeviceTimestamp;
};

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
