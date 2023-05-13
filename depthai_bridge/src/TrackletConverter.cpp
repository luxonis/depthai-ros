#include "depthai_bridge/TrackletConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"

namespace dai {
namespace ros {

TrackletConverter::TrackletConverter(std::string frameName, int width, int height, bool normalized, bool getBaseDeviceTimestamp)
    : _frameName(frameName),
      _width(width),
      _height(height),
      _normalized(normalized),
      _steadyBaseTime(std::chrono::steady_clock::now()),
      _getBaseDeviceTimestamp(getBaseDeviceTimestamp) {
    _rosBaseTime = ::ros::Time::now();
}

void TrackletConverter::toRosMsg(std::shared_ptr<dai::Tracklets> trackData, std::deque<TrackletMessages::TrackletArray>& trackletMsgs) {
    // setting the header
    std::chrono::_V2::steady_clock::time_point tstamp;
    if(_getBaseDeviceTimestamp)
        tstamp = trackData->getTimestampDevice();
    else
        tstamp = trackData->getTimestamp();

    TrackletMessages::TrackletArray trackletsMsg;

    trackletsMsg.header.stamp = getFrameTime(_rosBaseTime, _steadyBaseTime, tstamp);
    trackletsMsg.header.frame_id = _frameName;
    trackletsMsg.tracklets.resize(trackData->tracklets.size());

    // publishing
    for(int i = 0; i < trackData->tracklets.size(); ++i) {
        int xMin, yMin, xMax, yMax;
        if(_normalized) {
            xMin = trackData->tracklets[i].srcImgDetection.xmin;
            yMin = trackData->tracklets[i].srcImgDetection.ymin;
            xMax = trackData->tracklets[i].srcImgDetection.xmax;
            yMax = trackData->tracklets[i].srcImgDetection.ymax;
        } else {
            xMin = trackData->tracklets[i].srcImgDetection.xmin * _width;
            yMin = trackData->tracklets[i].srcImgDetection.ymin * _height;
            xMax = trackData->tracklets[i].srcImgDetection.xmax * _width;
            yMax = trackData->tracklets[i].srcImgDetection.ymax * _height;
        }

        float xSize = xMax - xMin;
        float ySize = yMax - yMin;
        float xCenter = xMin + xSize / 2;
        float yCenter = yMin + ySize / 2;

        trackletsMsg.tracklets[i].roi.center.x = xCenter;
        trackletsMsg.tracklets[i].roi.center.y = yCenter;
        trackletsMsg.tracklets[i].roi.size_x = xSize;
        trackletsMsg.tracklets[i].roi.size_y = ySize;

        trackletsMsg.tracklets[i].id = trackData->tracklets[i].id;
        trackletsMsg.tracklets[i].label = trackData->tracklets[i].label;
        trackletsMsg.tracklets[i].age = trackData->tracklets[i].age;
        trackletsMsg.tracklets[i].status = static_cast<std::underlying_type<dai::Tracklet::TrackingStatus>::type>(trackData->tracklets[i].status);

        trackletsMsg.tracklets[i].srcImgDetection.confidence = trackData->tracklets[i].srcImgDetection.confidence;
        trackletsMsg.tracklets[i].srcImgDetection.label = trackData->tracklets[i].srcImgDetection.label;
        trackletsMsg.tracklets[i].srcImgDetection.xmin = xMin;
        trackletsMsg.tracklets[i].srcImgDetection.xmax = xMax;
        trackletsMsg.tracklets[i].srcImgDetection.ymin = yMin;
        trackletsMsg.tracklets[i].srcImgDetection.ymax = yMax;

        // converting mm to meters since per ros rep-103 lenght should always be in meters
        trackletsMsg.tracklets[i].spatialCoordinates.x = trackData->tracklets[i].spatialCoordinates.x / 1000;
        trackletsMsg.tracklets[i].spatialCoordinates.y = trackData->tracklets[i].spatialCoordinates.y / 1000;
        trackletsMsg.tracklets[i].spatialCoordinates.z = trackData->tracklets[i].spatialCoordinates.z / 1000;
    }

    trackletMsgs.push_back(trackletsMsg);
}

TrackletArrayPtr TrackletConverter::toRosMsgPtr(std::shared_ptr<dai::Tracklets> trackData) {
    std::deque<TrackletMessages::TrackletArray> msgQueue;
    toRosMsg(trackData, msgQueue);
    auto msg = msgQueue.front();
    TrackletArrayPtr ptr = boost::make_shared<TrackletMessages::TrackletArray>(msg);
    return ptr;
}

}  // namespace ros
}  // namespace dai