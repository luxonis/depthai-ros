#pragma once

#include <deque>
#include <memory>
#include <string>

#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "depthai_bridge/BaseConverter.hpp"
#include "depthai_ros_msgs/msg/spatial_detection_array.hpp"
#include "rclcpp/time.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace depthai_bridge {

namespace SpatialMessages = depthai_ros_msgs::msg;
using SpatialDetectionArrayPtr = SpatialMessages::SpatialDetectionArray::SharedPtr;

class SpatialDetectionConverter : public BaseConverter {
   public:
    explicit SpatialDetectionConverter(std::string frameName, bool normalized = false, bool getBaseDeviceTimestamp = false);
    ~SpatialDetectionConverter();

    void toRosMsg(std::shared_ptr<dai::SpatialImgDetections> inNetData, std::deque<SpatialMessages::SpatialDetectionArray>& opDetectionMsg);
    void toRosVisionMsg(std::shared_ptr<dai::SpatialImgDetections> inNetData, std::deque<vision_msgs::msg::Detection3DArray>& opDetectionMsg);

    SpatialDetectionArrayPtr toRosMsgPtr(std::shared_ptr<dai::SpatialImgDetections> inNetData);

   private:
    bool normalized;
};

}  // namespace depthai_bridge
