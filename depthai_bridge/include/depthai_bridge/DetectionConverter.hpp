#include "depthai/depthai.hpp"
#include "sensor_msgs/Image.h"
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_ros_msgs/DetectionDaiArray.h>
#include <vision_msgs/Detection2DArray.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

namespace dai::rosBridge {
template <class rosMsg> class DetectionConverter {
public:
  // DetectionConverter() = default;
  DetectionConverter(std::string frameName, int width, int height, bool normalized = false);

  void toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData,
           rosMsg &opDetectionMsg, TimePoint tStamp, int32_t sequenceNum = -1);

  void toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData,
           rosMsg &opDetectionMsg);

boost::shared_ptr<rosMsg> toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData);

private:
  uint32_t _sequenceNum;
  int _width, _height;
  const std::string _frameName;
  bool _normalized;

};
}   // namespace dai::rosBridge