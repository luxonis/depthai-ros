#include "depthai_bridge/TrackDetectionConverter.hpp"

#include "depthai/depthai.hpp"
#include "depthai_bridge/depthaiUtility.hpp"

namespace dai {

namespace ros {

TrackDetectionConverter::TrackDetectionConverter(std::string frameName, int width, int height, bool normalized, float thresh, bool getBaseDeviceTimestamp)
    : _frameName(frameName),
      _width(width),
      _height(height),
      _normalized(normalized),
      _thresh(thresh),
      _steadyBaseTime(std::chrono::steady_clock::now()),
      _getBaseDeviceTimestamp(getBaseDeviceTimestamp) {
    _rosBaseTime = rclcpp::Clock().now();
}

TrackDetectionConverter::~TrackDetectionConverter() = default;

void TrackDetectionConverter::updateRosBaseTime() {
    updateBaseTime(_steadyBaseTime, _rosBaseTime, _totalNsChange);
}

void TrackDetectionConverter::toRosMsg(std::shared_ptr<dai::Tracklets> trackData, std::deque<depthai_ros_msgs::msg::TrackDetection2DArray>& opDetectionMsgs) {
    // setting the header
    std::chrono::_V2::steady_clock::time_point tstamp;
    if(_getBaseDeviceTimestamp)
        tstamp = trackData->getTimestampDevice();
    else
        tstamp = trackData->getTimestamp();

    depthai_ros_msgs::msg::TrackDetection2DArray opDetectionMsg;
    opDetectionMsg.header.stamp = getFrameTime(_rosBaseTime, _steadyBaseTime, tstamp);
    opDetectionMsg.header.frame_id = _frameName;
    opDetectionMsg.detections.resize(trackData->tracklets.size());

    // publishing
    for(int i = 0; i < trackData->tracklets.size(); ++i) {
        dai::Tracklet t = trackData->tracklets[i];
        dai::Rect roi;
        float xMin, yMin, xMax, yMax;

        if(_normalized)
            roi = t.roi;
        else
            roi = t.roi.denormalize(_width, _height);

        xMin = roi.topLeft().x;
        yMin = roi.topLeft().y;
        xMax = roi.bottomRight().x;
        yMax = roi.bottomRight().y;

        float xSize = xMax - xMin;
        float ySize = yMax - yMin;
        float xCenter = xMin + xSize / 2.;
        float yCenter = yMin + ySize / 2.;

        opDetectionMsg.detections[i].results.resize(1);

        opDetectionMsg.detections[i].results[0].hypothesis.class_id = std::to_string(t.label);
        opDetectionMsg.detections[i].results[0].hypothesis.score = _thresh;

        opDetectionMsg.detections[i].bbox.center.position.x = xCenter;
        opDetectionMsg.detections[i].bbox.center.position.y = yCenter;
        opDetectionMsg.detections[i].bbox.size_x = xSize;
        opDetectionMsg.detections[i].bbox.size_y = ySize;

        opDetectionMsg.detections[i].is_tracking = true;
        std::stringstream track_id_str;
        track_id_str << "" << t.id;
        opDetectionMsg.detections[i].tracking_id = track_id_str.str();
        opDetectionMsg.detections[i].tracking_age = t.age;
        opDetectionMsg.detections[i].tracking_status = static_cast<int32_t>(t.status);
    }

    opDetectionMsgs.push_back(opDetectionMsg);
}

depthai_ros_msgs::msg::TrackDetection2DArray::SharedPtr TrackDetectionConverter::toRosMsgPtr(std::shared_ptr<dai::Tracklets> trackData) {
    std::deque<depthai_ros_msgs::msg::TrackDetection2DArray> msgQueue;
    toRosMsg(trackData, msgQueue);
    auto msg = msgQueue.front();

    depthai_ros_msgs::msg::TrackDetection2DArray::SharedPtr ptr = std::make_shared<depthai_ros_msgs::msg::TrackDetection2DArray>(msg);

    return ptr;
}

}  // namespace ros

}  // namespace dai