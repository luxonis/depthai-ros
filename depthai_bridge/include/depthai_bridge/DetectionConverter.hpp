#include "depthai/depthai.hpp"
#include "sensor_msgs/Image.h"
#include <depthai_ros_msgs/DetectionDaiArray.h>
#include <vision_msgs/Detection2DArray.h>

namespace dai::ros {
template <class rosMsg> class DetectionConverter {
public:
  // DetectionConverter() = default;
  DetectionConverter(std::string frameName, int width, int height,
                     rosMsg &rosMsg, bool normalized = false);

  toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData,
           rosMsg &opDetectionMsg, timePoint tStamp, uint32_t sequenceNum = -1,
           std::string trackingId = "");

  toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData,
           rosMsg &opDetectionMsg);

  rosMsg::SharedPtr toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData);

private:
  uint32_t _sequenceNum;
  int _width, _height;
  const std::string _frameName;
  bool _normalized;
}

} // namespace dai::ros