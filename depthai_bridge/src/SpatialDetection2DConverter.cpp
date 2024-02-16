#include "depthai_bridge/SpatialDetection2DConverter.hpp"

#include "depthai_bridge/depthaiUtility.hpp"
#include "depthai/depthai.hpp"

namespace dai {

namespace ros {

SpatialDetection2DConverter::SpatialDetection2DConverter(std::string frameName, int width, int height, bool normalized, bool getBaseDeviceTimestamp)
    :	_frameName(frameName),
        _width(width),
        _height(height),
        _normalized(normalized),
        _steadyBaseTime(std::chrono::steady_clock::now()),
        _getBaseDeviceTimestamp(getBaseDeviceTimestamp) {
        _rosBaseTime = rclcpp::Clock().now();
}

SpatialDetection2DConverter::~SpatialDetection2DConverter() = default;

void SpatialDetection2DConverter::toRosVisionMsg(
    std::shared_ptr<dai::SpatialImgDetections> inNetData,
    std::deque<VisionMsgs::Detection2DArray>& opDetectionMsgs) {

    std::chrono::_V2::steady_clock::time_point tstamp;
    if(_getBaseDeviceTimestamp)
        tstamp = inNetData->getTimestampDevice();
    else
        tstamp = inNetData->getTimestamp();

    VisionMsgs::Detection2DArray opDetectionMsg;
    opDetectionMsg.header.stamp = getFrameTime(_rosBaseTime, _steadyBaseTime, tstamp);
    opDetectionMsg.header.frame_id = _frameName;
    opDetectionMsg.detections.resize(inNetData->detections.size());

    // publishing
    for (int i = 0; i < inNetData->detections.size(); ++i)
    {
        float xMin, yMin, xMax, yMax;
        if(_normalized)
        {
            xMin = inNetData->detections[i].xmin;
            yMin = inNetData->detections[i].ymin;
            xMax = inNetData->detections[i].xmax;
            yMax = inNetData->detections[i].ymax;
        }
        else
        {
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

        opDetectionMsg.detections[i].results[0].id = std::to_string(inNetData->detections[i].label);
        opDetectionMsg.detections[i].results[0].score = inNetData->detections[i].confidence;
        opDetectionMsg.detections[i].bbox.center.x = xCenter;
        opDetectionMsg.detections[i].bbox.center.y = yCenter;
        opDetectionMsg.detections[i].bbox.size_x = xSize;
        opDetectionMsg.detections[i].bbox.size_y = ySize;

        // converting mm to meters since per ros rep-103 lenght should always be in meters
        opDetectionMsg.detections[i].results[0].pose.pose.position.x = inNetData->detections[i].spatialCoordinates.x / 1000.;
        opDetectionMsg.detections[i].results[0].pose.pose.position.y = inNetData->detections[i].spatialCoordinates.y / 1000.;
        opDetectionMsg.detections[i].results[0].pose.pose.position.z = inNetData->detections[i].spatialCoordinates.z / 1000.;
    }

    opDetectionMsgs.push_back(opDetectionMsg);
}

}  // namespace ros

}  // namespace dai