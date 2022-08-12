#include <depthai_bridge/ImgDetectionConverter.hpp>

namespace dai {

namespace ros {

ImgDetectionConverter::ImgDetectionConverter(std::string frameName, int width, int height, bool normalized)
    : _frameName(frameName), _width(width), _height(height), _normalized(normalized), _steadyBaseTime(std::chrono::steady_clock::now()) {
#ifdef IS_ROS2
    _rosBaseTime = rclcpp::Clock().now();
#else
    _rosBaseTime = ::ros::Time::now();
#endif
}

void ImgDetectionConverter::toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData, std::deque<VisionMsgs::Detection2DArray>& opDetectionMsgs) {
    // setting the header
    auto tstamp = inNetData->getTimestamp();
    VisionMsgs::Detection2DArray opDetectionMsg;
/* #ifndef IS_ROS2
    auto rosNow = ::ros::Time::now();
    auto steadyTime = std::chrono::steady_clock::now();
    auto diffTime = steadyTime - tstamp;
    uint64_t nsec = rosNow.toNSec() - diffTime.count();
    auto rosStamp = rosNow.fromNSec(nsec);
    opDetectionMsg.header.stamp = rosStamp;
    opDetectionMsg.header.seq = inNetData->getSequenceNum();
#else
    auto rclNow = rclcpp::Clock().now();
    auto steadyTime = std::chrono::steady_clock::now();
    auto diffTime = steadyTime - tstamp;
    auto rclStamp = rclNow - diffTime;
    opDetectionMsg.header.stamp = rclStamp;
#endif */

    opDetectionMsg.header.stamp = getFrameTime(_rosBaseTime, _steadyBaseTime, tstamp);
    opDetectionMsg.header.frame_id = _frameName;
    opDetectionMsg.detections.resize(inNetData->detections.size());

    // TODO(Sachin): check if this works fine for normalized detection
    // publishing
    for(int i = 0; i < inNetData->detections.size(); ++i) {
        int xMin, yMin, xMax, yMax;
        if(_normalized) {
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

#if defined(IS_GALACTIC) || defined(IS_HUMBLE)
        opDetectionMsg.detections[i].id = std::to_string(inNetData->detections[i].label);
        opDetectionMsg.detections[i].results[0].hypothesis.class_id = std::to_string(inNetData->detections[i].label);
        opDetectionMsg.detections[i].results[0].hypothesis.score = inNetData->detections[i].confidence;
#elif IS_ROS2
        opDetectionMsg.detections[i].results[0].id = std::to_string(inNetData->detections[i].label);
        opDetectionMsg.detections[i].results[0].score = inNetData->detections[i].confidence;
#else
        opDetectionMsg.detections[i].results[0].id = inNetData->detections[i].label;
        opDetectionMsg.detections[i].results[0].score = inNetData->detections[i].confidence;
#endif

#ifdef IS_HUMBLE
        opDetectionMsg.detections[i].bbox.center.position.x = xCenter;
        opDetectionMsg.detections[i].bbox.center.position.y = yCenter;
#else
        opDetectionMsg.detections[i].bbox.center.x = xCenter;
        opDetectionMsg.detections[i].bbox.center.y = yCenter;
#endif
        opDetectionMsg.detections[i].bbox.size_x = xSize;
        opDetectionMsg.detections[i].bbox.size_y = ySize;
    }

    opDetectionMsgs.push_back(opDetectionMsg);
}

Detection2DArrayPtr ImgDetectionConverter::toRosMsgPtr(std::shared_ptr<dai::ImgDetections> inNetData) {
    std::deque<VisionMsgs::Detection2DArray> msgQueue;
    toRosMsg(inNetData, msgQueue);
    auto msg = msgQueue.front();
#ifdef IS_ROS2
    Detection2DArrayPtr ptr = std::make_shared<VisionMsgs::Detection2DArray>(msg);
#else
    Detection2DArrayPtr ptr = boost::make_shared<VisionMsgs::Detection2DArray>(msg);
#endif
    return ptr;
}

}  // namespace ros
}  // namespace dai