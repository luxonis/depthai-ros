#pragma once

#include <deque>
#include <memory>
#include <string>

#include "depthai/pipeline/datatype/Tracklets.hpp"
#include "depthai_bridge/BaseConverter.hpp"
#include "depthai_ros_msgs/msg/track_detection2_d_array.hpp"
#include "rclcpp/time.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace depthai_bridge {

class TrackSpatialDetectionConverter : public BaseConverter {
   public:
    TrackSpatialDetectionConverter(
        std::string frameName, int width, int height, bool normalized = false, float thresh = 0.0, bool getBaseDeviceTimestamp = false);
    ~TrackSpatialDetectionConverter();

    void toRosMsg(std::shared_ptr<dai::Tracklets> trackData, std::deque<depthai_ros_msgs::msg::TrackDetection2DArray>& opDetectionMsgs);

    depthai_ros_msgs::msg::TrackDetection2DArray::SharedPtr toRosMsgPtr(std::shared_ptr<dai::Tracklets> trackData);

   private:
    int width, height;
    bool normalized;
    float thresh;
};

}  // namespace depthai_bridge
