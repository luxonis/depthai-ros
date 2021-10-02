#include <depthai_bridge/ImageConverter.hpp>
#include "depthai/depthai.hpp"

#ifdef IS_ROS2
    #include <vision_msgs/msg/Detection2DArray.h>
    namespace VisionMsgs = vision_msgs::msg;
#else
    #include <vision_msgs/Detection2DArray.h>
    #include <boost/make_shared.hpp>
    #include <boost/shared_ptr.hpp>
    namespace VisionMsgs = vision_msgs;
#endif

namespace dai::rosBridge {

class ImgDetectionConverter {
   public:
    // DetectionConverter() = default;
    ImgDetectionConverter(std::string frameName, int width, int height, bool normalized = false);

    void toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData, VisionMsgs::Detection2DArray& opDetectionMsg, TimePoint tStamp, int32_t sequenceNum = -1);

    void toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData, VisionMsgs::Detection2DArray& opDetectionMsg);

    VisionMsgs::Detection2DArray::Ptr toRosMsgPtr(std::shared_ptr<dai::ImgDetections> inNetData);

   private:
    uint32_t _sequenceNum;
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

}  // namespace dai::rosBridge