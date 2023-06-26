#include "depthai_bridge/SpatialDetectionConverter.hpp"

#include "depthai_bridge/depthaiUtility.hpp"

namespace dai {
namespace ros {

SpatialDetectionConverter::SpatialDetectionConverter(std::string frameName, int width, int height, bool normalized, bool getBaseDeviceTimestamp)
    : _frameName(frameName),
      _width(width),
      _height(height),
      _normalized(normalized),
      _steadyBaseTime(std::chrono::steady_clock::now()),
      _getBaseDeviceTimestamp(getBaseDeviceTimestamp) {
    _rosBaseTime = ::ros::Time::now();
}

void SpatialDetectionConverter::updateRosBaseTime() {
    updateBaseTime(_steadyBaseTime, _rosBaseTime, _totalNsChange);
}

void SpatialDetectionConverter::toRosMsg(std::shared_ptr<dai::SpatialImgDetections> inNetData,
                                         std::deque<SpatialMessages::SpatialDetectionArray>& opDetectionMsgs) {
    if(_updateRosBaseTimeOnToRosMsg) {
        updateRosBaseTime();
    }
    // setting the header
    std::chrono::_V2::steady_clock::time_point tstamp;
    if(_getBaseDeviceTimestamp)
        tstamp = inNetData->getTimestampDevice();
    else
        tstamp = inNetData->getTimestamp();
    SpatialMessages::SpatialDetectionArray opDetectionMsg;

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

        opDetectionMsg.detections[i].results[0].id = inNetData->detections[i].label;
        opDetectionMsg.detections[i].results[0].score = inNetData->detections[i].confidence;

        opDetectionMsg.detections[i].bbox.center.x = xCenter;
        opDetectionMsg.detections[i].bbox.center.y = yCenter;
        opDetectionMsg.detections[i].bbox.size_x = xSize;
        opDetectionMsg.detections[i].bbox.size_y = ySize;

        // converting mm to meters since per ros rep-103 lenght should always be in meters
        opDetectionMsg.detections[i].position.x = inNetData->detections[i].spatialCoordinates.x / 1000;
        opDetectionMsg.detections[i].position.y = inNetData->detections[i].spatialCoordinates.y / 1000;
        opDetectionMsg.detections[i].position.z = inNetData->detections[i].spatialCoordinates.z / 1000;
    }

    opDetectionMsgs.push_back(opDetectionMsg);
}

void SpatialDetectionConverter::toRosVisionMsg(std::shared_ptr<dai::SpatialImgDetections> inNetData,
                                               std::deque<vision_msgs::Detection3DArray>& opDetectionMsgs) {
    if(_updateRosBaseTimeOnToRosMsg) {
        updateRosBaseTime();
    }
    std::chrono::_V2::steady_clock::time_point tstamp;
    if(_getBaseDeviceTimestamp)
        tstamp = inNetData->getTimestampDevice();
    else
        tstamp = inNetData->getTimestamp();
    vision_msgs::Detection3DArray opDetectionMsg;

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

        opDetectionMsg.detections[i].results[0].id = inNetData->detections[i].label;
        opDetectionMsg.detections[i].results[0].score = inNetData->detections[i].confidence;

        opDetectionMsg.detections[i].bbox.center.position.x = xCenter;
        opDetectionMsg.detections[i].bbox.center.position.y = yCenter;
        opDetectionMsg.detections[i].bbox.size.x = xSize;
        opDetectionMsg.detections[i].bbox.size.y = ySize;

        // converting mm to meters since per ros rep-103 lenght should always be in meters
        opDetectionMsg.detections[i].results[0].pose.pose.position.x = inNetData->detections[i].spatialCoordinates.x / 1000;
        opDetectionMsg.detections[i].results[0].pose.pose.position.y = inNetData->detections[i].spatialCoordinates.y / 1000;
        opDetectionMsg.detections[i].results[0].pose.pose.position.z = inNetData->detections[i].spatialCoordinates.z / 1000;
    }

    opDetectionMsgs.push_back(opDetectionMsg);
}

SpatialDetectionArrayPtr SpatialDetectionConverter::toRosMsgPtr(std::shared_ptr<dai::SpatialImgDetections> inNetData) {
    std::deque<SpatialMessages::SpatialDetectionArray> msgQueue;
    toRosMsg(inNetData, msgQueue);
    auto msg = msgQueue.front();
    SpatialDetectionArrayPtr ptr = boost::make_shared<SpatialMessages::SpatialDetectionArray>(msg);
    return ptr;
}

}  // namespace ros
}  // namespace dai