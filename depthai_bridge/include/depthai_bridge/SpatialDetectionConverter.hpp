#include "depthai/depthai.hpp"
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_ros_msgs/msg/spatial_detection_array.hpp>

namespace dai::rosBridge {

class SpatialDetectionConverter {
public:
  // DetectionConverter() = default;
  SpatialDetectionConverter(std::string frameName, int width, int height, bool normalized = false);

  void toRosMsg(std::shared_ptr<dai::SpatialImgDetections> inNetData,
           depthai_ros_msgs::msg::SpatialDetectionArray &opDetectionMsg, TimePoint tStamp);

  void toRosMsg(std::shared_ptr<dai::SpatialImgDetections> inNetData,
           depthai_ros_msgs::msg::SpatialDetectionArray &opDetectionMsg);

depthai_ros_msgs::msg::SpatialDetectionArray::SharedPtr toRosMsgPtr(std::shared_ptr<dai::SpatialImgDetections> inNetData);

private:
  int _width, _height;
  const std::string _frameName;
  bool _normalized;
};

  /** TODO(sachin): Do we need to have ros msg -> dai bounding box ?
   * is there any situation where we would need to have xlinkin to take bounding
   * box as input. One scenario would to take this as input and use ImageManip
   * node to crop the roi of the image. Since it is not available yet. Leaving
   * it out for now to speed up on other tasks feel free to raise a issue if you
   * feel that feature is good to have...
   */

}   // namespace dai::rosBridge