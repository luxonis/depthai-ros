#pragma once

#include <deque>
#include <memory>
#include <string>

#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai_bridge/BaseConverter.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"

namespace depthai_bridge {

namespace DisparityMsgs = stereo_msgs::msg;
namespace ImageMsgs = sensor_msgs::msg;
using ImagePtr = ImageMsgs::Image::SharedPtr;
using DisparityImagePtr = DisparityMsgs::DisparityImage::SharedPtr;

using TimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>;

class DisparityConverter : public BaseConverter {
   public:
    /**
     * @param frameName frame_id of the messages
     * @param focalLength focal length of the (rectified) camera, in pixels
     * @param baseline distance between the stereo cameras, in centimeters
     * @param minDepth minimum depth, in centimeters
     * @param maxDepth maximum depth, in centimeters
     */
    DisparityConverter(
        std::string frameName, float focalLength, float baseline = 7.5, float minDepth = 80, float maxDepth = 1100, bool getBaseDeviceTimestamp = false);
    ~DisparityConverter();

    void toRosMsg(std::shared_ptr<dai::ImgFrame> inData, std::deque<DisparityMsgs::DisparityImage>& outImageMsg);
    DisparityImagePtr toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData);

    float getFocalLength() const;
    float getBaseline() const;
    float getMinDepth() const;
    float getMaxDepth() const;

   private:
    const float focalLength = 882.2, baseline = 7.5, minDepth = 80, maxDepth;
};

}  // namespace depthai_bridge
