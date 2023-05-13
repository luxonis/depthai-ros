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
    TrackletConverter(std::string frameName, int width, int height, bool normalized = false, bool getBaseDeviceTimestamp = false);
    void toRosMsg(std::shared_ptr<dai::Tracklets> trackData, std::deque<TrackletMessages::TrackletArray>& trackletsMsg);
    void toRosVisionMsg(std::shared_ptr<dai::Tracklets> trackData, std::deque<vision_msgs::Detection3DArray>& trackletsMsg);
    TrackletArrayPtr toRosMsgPtr(std::shared_ptr<dai::Tracklets> trackData);

   private:
    int _width, _height;
    const std::string _frameName;
    bool _normalized;
    std::chrono::time_point<std::chrono::steady_clock> _steadyBaseTime;
    ::ros::Time _rosBaseTime;
    bool _getBaseDeviceTimestamp;
};

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
