#include "depthai_bridge/ImgDetectionConverter.hpp"

#include "depthai_bridge/depthaiUtility.hpp"

namespace dai {

namespace ros {

ImgDetectionConverter::ImgDetectionConverter(std::string frameName, int width, int height, bool normalized, bool getBaseDeviceTimestamp)
    : frameName(frameName),
      width(width),
      height(height),
      normalized(normalized),
      steadyBaseTime(std::chrono::steady_clock::now()),
      getBaseDeviceTimestamp(getBaseDeviceTimestamp) {
    rosBaseTime = rclcpp::Clock().now();
}

ImgDetectionConverter::~ImgDetectionConverter() = default;

void ImgDetectionConverter::updateRosBaseTime() {
    updateBaseTime(steadyBaseTime, rosBaseTime, totalNsChange);
}

void ImgDetectionConverter::toRosMsg(std::shared_ptr<dai::ImgDetections> inNetData, std::deque<VisionMsgs::Detection2DArray>& opDetectionMsgs) {
    if(updateRosBaseTimeOnToRosMsg) {
        updateRosBaseTime();
    }
    std::chrono::_V2::steady_clock::time_point tstamp;
    if(getBaseDeviceTimestamp)
        tstamp = inNetData->getTimestampDevice();
    else
        tstamp = inNetData->getTimestamp();

    VisionMsgs::Detection2DArray opDetectionMsg;

    opDetectionMsg.header.stamp = getFrameTime(rosBaseTime, steadyBaseTime, tstamp);
    opDetectionMsg.header.frame_id = frameName;
    opDetectionMsg.detections.resize(inNetData->detections.size());

    // TODO(Sachin): check if this works fine for normalized detection
    // publishing
    for(int i = 0; i < inNetData->detections.size(); ++i) {
        int xMin, yMin, xMax, yMax;
        if(normalized) {
            xMin = inNetData->detections[i].xmin;
            yMin = inNetData->detections[i].ymin;
            xMax = inNetData->detections[i].xmax;
            yMax = inNetData->detections[i].ymax;
        } else {
            xMin = inNetData->detections[i].xmin * width;
            yMin = inNetData->detections[i].ymin * height;
            xMax = inNetData->detections[i].xmax * width;
            yMax = inNetData->detections[i].ymax * height;
        }

        float xSize = xMax - xMin;
        float ySize = yMax - yMin;
        float xCenter = xMin + xSize / 2;
        float yCenter = yMin + ySize / 2;

        opDetectionMsg.detections[i].results.resize(1);

        opDetectionMsg.detections[i].id = std::to_string(inNetData->detections[i].label);
        opDetectionMsg.detections[i].results[0].hypothesis.class_id = std::to_string(inNetData->detections[i].label);
        opDetectionMsg.detections[i].results[0].hypothesis.score = inNetData->detections[i].confidence;
        opDetectionMsg.detections[i].bbox.center.position.x = xCenter;
        opDetectionMsg.detections[i].bbox.center.position.y = yCenter;
        opDetectionMsg.detections[i].bbox.size_x = xSize;
        opDetectionMsg.detections[i].bbox.size_y = ySize;
    }

    opDetectionMsgs.push_back(opDetectionMsg);
}

Detection2DArrayPtr ImgDetectionConverter::toRosMsgPtr(std::shared_ptr<dai::ImgDetections> inNetData) {
    std::deque<VisionMsgs::Detection2DArray> msgQueue;
    toRosMsg(inNetData, msgQueue);
    auto msg = msgQueue.front();
    Detection2DArrayPtr ptr = std::make_shared<VisionMsgs::Detection2DArray>(msg);
    return ptr;
}

}  // namespace ros
}  // namespace dai
