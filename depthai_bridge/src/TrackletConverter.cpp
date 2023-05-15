#include "depthai_bridge/TrackletConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"

namespace dai {
namespace ros {

TrackletConverter::TrackletConverter(std::string frameName,
                                     int width_srcImgDetection,
                                     int height_srcImgDetection,
                                     int width_srcImgTracker,
                                     int height_srcImgTracker,
                                     bool normalized_srcImgDetection,
                                     bool normalized_srcImgTracker,
                                     bool getBaseDeviceTimestamp)
    : _frameName(frameName),
      _width_srcImgDetection(width_srcImgDetection),
      _height_srcImgDetection(height_srcImgDetection),
      _width_srcImgTracker(width_srcImgTracker),
      _height_srcImgTracker(height_srcImgTracker),
      _normalized_srcImgDetection(normalized_srcImgDetection),
      _normalized_srcImgTracker(normalized_srcImgTracker),
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
        // srcImgDetection -> inputDetectionsFrame
        int xMin_srcImgDetection, yMin_srcImgDetection, xMax_srcImgDetection, yMax_srcImgDetection;
        if(_normalized_srcImgDetection) {
            xMin_srcImgDetection = trackData->tracklets[i].srcImgDetection.xmin;
            yMin_srcImgDetection = trackData->tracklets[i].srcImgDetection.ymin;
            xMax_srcImgDetection = trackData->tracklets[i].srcImgDetection.xmax;
            yMax_srcImgDetection = trackData->tracklets[i].srcImgDetection.ymax;
        } else {
            xMin_srcImgDetection = trackData->tracklets[i].srcImgDetection.xmin * _width_srcImgDetection;
            yMin_srcImgDetection = trackData->tracklets[i].srcImgDetection.ymin * _height_srcImgDetection;
            xMax_srcImgDetection = trackData->tracklets[i].srcImgDetection.xmax * _width_srcImgDetection;
            yMax_srcImgDetection = trackData->tracklets[i].srcImgDetection.ymax * _height_srcImgDetection;
        }
        auto xSize_srcImgDetection = xMax_srcImgDetection - xMin_srcImgDetection;
        auto ySize_srcImgDetection = yMax_srcImgDetection - yMin_srcImgDetection;
        auto xCenter_srcImgDetection = xMin_srcImgDetection + xSize_srcImgDetection / 2;
        auto yCenter_srcImgDetection = yMin_srcImgDetection + ySize_srcImgDetection / 2;

        // roi -> inputTrackerFrame
        dai::Rect roi;
        if(_normalized_srcImgTracker) {
            roi = trackData->tracklets[i].roi;
        } else {
            roi = trackData->tracklets[i].roi.denormalize(_width_srcImgTracker, _height_srcImgTracker);
        }
        auto xSize_srcImgTracker = roi.size().width;
        auto ySize_srcImgTracker = roi.size().height;
        auto xCenter_srcImgTracker = roi.topLeft().x + roi.width / 2;
        auto yCenter_srcImgTracker = roi.topLeft().y + roi.height / 2;

        trackletsMsg.tracklets[i].roi.center.x = xCenter_srcImgTracker;
        trackletsMsg.tracklets[i].roi.center.y = yCenter_srcImgTracker;
        trackletsMsg.tracklets[i].roi.size_x = xSize_srcImgTracker;
        trackletsMsg.tracklets[i].roi.size_y = ySize_srcImgTracker;

        trackletsMsg.tracklets[i].id = trackData->tracklets[i].id;
        trackletsMsg.tracklets[i].label = trackData->tracklets[i].label;
        trackletsMsg.tracklets[i].age = trackData->tracklets[i].age;
        trackletsMsg.tracklets[i].status = static_cast<std::underlying_type<dai::Tracklet::TrackingStatus>::type>(trackData->tracklets[i].status);

        trackletsMsg.tracklets[i].srcImgDetectionHypothesis.score = trackData->tracklets[i].srcImgDetection.confidence;
        trackletsMsg.tracklets[i].srcImgDetectionHypothesis.id = trackData->tracklets[i].srcImgDetection.label;
        trackletsMsg.tracklets[i].srcImgDetectionBBox.center.x = xCenter_srcImgDetection;
        trackletsMsg.tracklets[i].srcImgDetectionBBox.center.y = yCenter_srcImgDetection;
        trackletsMsg.tracklets[i].srcImgDetectionBBox.size_x = xSize_srcImgDetection;
        trackletsMsg.tracklets[i].srcImgDetectionBBox.size_y = ySize_srcImgDetection;

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