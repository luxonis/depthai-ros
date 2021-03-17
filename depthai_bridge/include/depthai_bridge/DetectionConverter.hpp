#include "depthai/depthai.hpp"
#include "sensor_msgs/msg/Image.hpp"
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_ros_msgs/msg/DetectionDaiArray.hpp>
#include <vision_msgs/msg/Detection2DArray.hpp>

// #include <boost/make_shared.hpp>
// #include <boost/shared_ptr.hpp>

namespace dai::rosBridge {

void detectionMsgHelper(depthai_ros_msgs::msgs::DetectionDai& rosDetection, dai::ImgDetection& daiDetection){
    rosDetection.position.x = daiDetection.xdepth;
    rosDetection.position.y = daiDetection.ydepth;
    rosDetection.position.z = daiDetection.zdepth;
}

void detectionMsgHelper(vision_msgs::msgs::Detection2D& rosDetection, dai::ImgDetection& daiDetection){}

template <class rosMsg> class DetectionConverter {
public:
  // DetectionConverter() = default;
  DetectionConverter(std::string frameName, int width, int height, bool normalized = false);

  void toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData,
           rosMsg &opDetectionMsg, TimePoint tStamp, int32_t sequenceNum = -1);

  void toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData,
           rosMsg &opDetectionMsg);

  std::shared_ptr<rosMsg> toRosMsgPtr(std::shared_ptr<dai::ImgDetections> inNetData);

private:
  uint32_t _sequenceNum;
  int _width, _height;
  const std::string _frameName;
  bool _normalized;
};

// -----------------------------DetectionConverter class below----------------------------------- //

template <class rosMsg>
DetectionConverter<rosMsg>::DetectionConverter(std::string frameName, int width,
                                       int height,  bool normalized)
    : _frameName(frameName), _width(width), _height(height),
      _normalized(normalized), _sequenceNum(0) {}

template <class rosMsg>
void DetectionConverter<rosMsg>::toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData,
                             rosMsg &opDetectionMsg, TimePoint tStamp,
                             int32_t sequenceNum) {

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

template <class rosMsg>
void DetectionConverter<rosMsg>::toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData,
                               rosMsg &opDetectionMsg) {

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
      opDetectionMsg.detections[i].results[0].id =
          std::to_string(inNetData->detections[i].label);
      opDetectionMsg.detections[i].results[0].score =
          inNetData->detections[i].confidence;

      opDetectionMsg.detections[i].bbox.center.x = xCenter;
      opDetectionMsg.detections[i].bbox.center.y = yCenter;
      opDetectionMsg.detections[i].bbox.size_x = xSize;
      opDetectionMsg.detections[i].bbox.size_y = ySize;
      // opDetectionMsg.detections[i].is_tracking = _isTracking;

      // if (isTracking) {
      // opDetectionMsg.detections[i].tracking_id = trackingId;
      // }
      detectionMsgHelper(opDetectionMsg.detections[i],
                         inNetData->detections[i]);

    }
    _sequenceNum++;
  }

template <class rosMsg>
boost::shared_ptr<rosMsg> DetectionConverter<rosMsg>::toRosMsgPtr(std::shared_ptr<dai::ImgDetections> inNetData){
    std::::shared_ptr<rosMsg> ptr = std::make_shared<rosMsg>();
    toRosMsg(inNetData, *ptr);
    return ptr;
  }

  /** TODO(sachin): Do we need to have ros msg -> dai bounding box ?
   * is there any situation where we would need to have xlinkin to take bounding
   * box as input. One scenario would to take this as input and use ImageManip
   * node to crop the roi of the image. Since it is not available yet. Leaving
   * it out for now to speed up on other tasks feel free to raise a issue if you
   * feel that feature is good to have...
   */

}   // namespace dai::rosBridge