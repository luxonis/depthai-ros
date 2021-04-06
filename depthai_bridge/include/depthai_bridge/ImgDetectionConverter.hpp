#include "depthai/depthai.hpp"
#include <depthai_bridge/ImageConverter.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

namespace dai::rosBridge {

class ImgDetectionConverter {
public:
  // DetectionConverter() = default;
  ImgDetectionConverter(std::string frameName, int width, int height, bool normalized = false);

  void toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData,
           vision_msgs::msg::Detection2DArray &opDetectionMsg, TimePoint tStamp);

  void toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData,
           vision_msgs::msg::Detection2DArray &opDetectionMsg);

vision_msgs::msg::Detection2DArray::SharedPtr toRosMsgPtr(std::shared_ptr<dai::ImgDetections> inNetData);

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