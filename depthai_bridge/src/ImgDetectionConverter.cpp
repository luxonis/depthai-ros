#include <depthai_bridge/ImgDetectionConverter.hpp>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

namespace dai::rosBridge{

ImgDetectionConverter::ImgDetectionConverter(std::string frameName, int width,
                                       int height, bool normalized)
    : _frameName(frameName), _width(width), _height(height),
      _normalized(normalized) {}

void ImgDetectionConverter::toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData,
                             vision_msgs::msg::Detection2DArray &opDetectionMsg, TimePoint tStamp) {

    toRosMsg(inNetData, opDetectionMsg);

    int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(
                      tStamp.time_since_epoch())
                      .count();
    int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       tStamp.time_since_epoch())
                       .count() %
                   1000000000UL;
    opDetectionMsg.header.stamp = rclcpp::Time(sec, nsec);
    opDetectionMsg.header.frame_id = _frameName;
  }

void ImgDetectionConverter::toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData,
                               vision_msgs::msg::Detection2DArray &opDetectionMsg) {

    // if (inNetData->detections.size() == 0)
    //   throw std::runtime_error(
    //       "Make sure to send the detections with size greater than 0");

    // setting the header
    opDetectionMsg.header.stamp = rclcpp::Clock().now();
    opDetectionMsg.header.frame_id = _frameName;

    opDetectionMsg.detections.resize(inNetData->detections.size());

    // TODO(Sachin): check if this works fine for normalized detection
    // publishing
    for (int i = 0; i < inNetData->detections.size(); ++i) {
      int xMin, yMin, xMax, yMax;
      if (_normalized) {
        xMin = inNetData->detections[i].xmin;
        yMin = inNetData->detections[i].ymin;
        xMax = inNetData->detections[i].xmax;
        yMax = inNetData->detections[i].ymax;
      } else {
        xMin = inNetData->detections[i].xmin * _width;
        yMin = inNetData->detections[i].ymin * _height;
        xMax = inNetData->detections[i].xmax * _width;
        yMax = inNetData->detections[i].ymax * _height;
      }

      float xSize = xMax - xMin;
      float ySize = yMax - yMin;
      float xCenter = xMin + xSize / 2;
      float yCenter = yMin + ySize / 2;

      opDetectionMsg.detections[i].results.resize(1);
      opDetectionMsg.detections[i].results[0].hypothesis.class_id =
          std::to_string(inNetData->detections[i].label);
      opDetectionMsg.detections[i].results[0].hypothesis.score =
          inNetData->detections[i].confidence;

      opDetectionMsg.detections[i].bbox.center.x = xCenter;
      opDetectionMsg.detections[i].bbox.center.y = yCenter;
      opDetectionMsg.detections[i].bbox.size_x = xSize;
      opDetectionMsg.detections[i].bbox.size_y = ySize;

    }
  }

vision_msgs::msg::Detection2DArray::SharedPtr ImgDetectionConverter::toRosMsgPtr(std::shared_ptr<dai::ImgDetections> inNetData){
    vision_msgs::msg::Detection2DArray::SharedPtr ptr = std::make_shared<vision_msgs::msg::Detection2DArray>();
    toRosMsg(inNetData, *ptr);
    return ptr;
  }

}  // namespace dai::rosBridge