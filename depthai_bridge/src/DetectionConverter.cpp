#include "depthai_bridge/DetectionConverter.hpp"

namespace dai::ros {

template <
    typename Msg,
    std::enable_if_t<std::is_same<Msg, depthai_ros_msgs::DetectionDai>::value,
                     bool> = true>
void detectionMsgHelper(Msg &rosDetection, dai::ImgDetection &daiDetection) {
  rosDetection.position.x = daiDetection.xdepth;
  rosDetection.position.y = daiDetection.ydepth;
  rosDetection.position.z = daiDetection.zdepth;
}

template <typename Msg,
          std::enable_if_t<
              not std::is_same<Msg, depthai_ros_msgs::DetectionDai>::value,
              bool> = true>
void detectionMsgHelper(Msg &rosDetection, dai::ImgDetection &daiDetection) {}

DetectionConverter::DetectionConverter(std::string frameName, int width,
                                       int height, T& rosMsg, bool normalized = false)
    : _frameName(frameName), _width(width), _height(height),
      _normalized(normalized), _sequenceNum(0) {}

DetectionConverter::toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData,
                             T &opDetectionMsg, timePoint tStamp,
                             uint32_t sequenceNum = -1) {

    toRosMsg(inNetData, opDetectionMsg);
    if (sequenceNum != -1)
      _sequenceNum = sequenceNum;

    int32_t sec = std::chrono::duration_cast<std::chrono::seconds>(
                      tStamp.time_since_epoch())
                      .count();
    int32_t nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       tStamp.time_since_epoch())
                       .count() %
                   1000000000UL;
    opDetectionMsg.header.seq = _sequenceNum;
    opDetectionMsg.header.stamp = ros::Time(sec, nsec);
    opDetectionMsg.header.frame_id = _frameName;
  }

  DetectionConverter::toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData,
                               T & opDetectionMsg) {

    // if (inNetData->detections.size() == 0)
    //   throw std::runtime_error(
    //       "Make sure to send the detections with size greater than 0");

    // setting the header
    opDetectionMsg.header.seq = _sequenceNum;
    opDetectionMsg.header.stamp = ros::Time::now();
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
        xMin = inNetData->detections[i].xmin * w;
        yMin = inNetData->detections[i].ymin * h;
        xMax = inNetData->detections[i].xmax * w;
        yMax = inNetData->detections[i].ymax * h;
      }

      float xSize = xMax - xMin;
      float ySize = yMax - yMin;
      float xCenter = xMin + xSize / 2;
      float yCenter = yMin + ySize / 2;

      opDetectionMsg.detections[i].results.resize(1);
      opDetectionMsg.detections[i].results[0].id =
          std::to_string(inNetData->detections[i].label);
      opDetectionMsg.detections[i].results[0].score =
          inNetData->detections[i].confidence;

      opDetectionMsg.detections[i].bbox.center.x = xCenter;
      opDetectionMsg.detections[i].bbox.center.y = yCenter;
      opDetectionMsg.detections[i].bbox.size_x = xSize;
      opDetectionMsg.detections[i].bbox.size_y = ySize;
      opDetectionMsg.detections[i].is_tracking = _isTracking;

      // if (isTracking) {
      // opDetectionMsg.detections[i].tracking_id = trackingId;
      // }
      detectionMsgHelper(opDetectionMsg.detections[i],
                         inNetData->detections[i]);
    }
    _sequenceNum++;
  }

  rosMsg::SharedPtr ImageConverter::toRosMsgPtr(std::shared_ptr<dai::ImgDetections> inNetData){
    rosMsg::SharedPtr ptr = boost::make_shared<rosMsg>();
    toRosMsg(inData, *ptr);
    return ptr;
  }

  /** TODO(sachin): Do we need to have ros msg -> dai bounding box ?
   * is there any situation where we would need to have xlinkin to take bounding
   * box as input. One scenario would to take this as input and use ImageManip
   * node to crop the roi of the image. Since it is not available yet. Leaving
   * it out for now to speed up on other tasks feel free to raise a issue if you
   * feel that feature is good to have...
   */

} // namespace dai::ros