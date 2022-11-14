#include <depthai_bridge/SpatialDetectionConverter.hpp>

namespace dai {
namespace ros {

SpatialDetectionConverter::SpatialDetectionConverter(std::string frameName, int width, int height, bool normalized)
    : _frameName(frameName), _width(width), _height(height), _normalized(normalized), _steadyBaseTime(std::chrono::steady_clock::now()) {}

void SpatialDetectionConverter::toRosMsg(std::shared_ptr<dai::SpatialImgDetections> inNetData,
                                         std::deque<SpatialMessages::SpatialDetectionArray>& opDetectionMsgs) {
    auto tstamp = inNetData->getTimestamp();
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

#if defined(IS_GALACTIC) || defined(IS_HUMBLE)
        opDetectionMsg.detections[i].results[0].class_id = std::to_string(inNetData->detections[i].label);
#elif IS_ROS2
        opDetectionMsg.detections[i].results[0].id = std::to_string(inNetData->detections[i].label);
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

        // converting mm to meters since per ros rep-103 lenght should always be in meters
        opDetectionMsg.detections[i].position.x = inNetData->detections[i].spatialCoordinates.x / 1000;
        opDetectionMsg.detections[i].position.y = inNetData->detections[i].spatialCoordinates.y / 1000;
        opDetectionMsg.detections[i].position.z = inNetData->detections[i].spatialCoordinates.z / 1000;
    }

    opDetectionMsgs.push_back(opDetectionMsg);
}

SpatialDetectionArrayPtr SpatialDetectionConverter::toRosMsgPtr(std::shared_ptr<dai::SpatialImgDetections> inNetData) {
    std::deque<SpatialMessages::SpatialDetectionArray> msgQueue;
    toRosMsg(inNetData, msgQueue);
    auto msg = msgQueue.front();
    SpatialDetectionArrayPtr ptr = std::make_shared<SpatialMessages::SpatialDetectionArray>(msg);
    return ptr;
}

}  // namespace ros
}  // namespace dai