#pragma once

#include <deque>
#include <memory>
#include <string>

#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai_bridge/BaseConverter.hpp"
#include "rclcpp/time.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace depthai_bridge {

namespace VisionMsgs = vision_msgs::msg;
using Detection2DArrayPtr = VisionMsgs::Detection2DArray::SharedPtr;

class ImgDetectionConverter : public BaseConverter {
   public:
    explicit ImgDetectionConverter(std::string frameName, bool normalized = false, bool getBaseDeviceTimestamp = false);
    ~ImgDetectionConverter();

    void toRosMsg(std::shared_ptr<dai::ImgDetections> inData, std::deque<VisionMsgs::Detection2DArray>& opDetectionMsgs);

    Detection2DArrayPtr toRosMsgPtr(std::shared_ptr<dai::ImgDetections> inNetData);

   private:
    bool normalized;
};

}  // namespace depthai_bridge
