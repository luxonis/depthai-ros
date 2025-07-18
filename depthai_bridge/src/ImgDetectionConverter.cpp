#include "depthai_bridge/ImgDetectionConverter.hpp"

namespace depthai_bridge {

ImgDetectionConverter::ImgDetectionConverter(std::string frameName, bool normalized, bool getBaseDeviceTimestamp)
    : BaseConverter(std::move(frameName), getBaseDeviceTimestamp), normalized(normalized) {}

ImgDetectionConverter::~ImgDetectionConverter() = default;

void ImgDetectionConverter::toRosMsg(std::shared_ptr<dai::ImgDetections> inData, std::deque<VisionMsgs::Detection2DArray>& opDetectionMsgs) {
    VisionMsgs::Detection2DArray opDetectionMsg;

    opDetectionMsg.header = getRosHeader(inData);
    opDetectionMsg.header.frame_id = frameName;
    opDetectionMsg.detections.resize(inData->detections.size());

    auto [width, height] = inData->transformation->getSize();
    for(int i = 0; i < inData->detections.size(); ++i) {
        int xMin, yMin, xMax, yMax;
        if(normalized) {
            xMin = inData->detections[i].xmin;
            yMin = inData->detections[i].ymin;
            xMax = inData->detections[i].xmax;
            yMax = inData->detections[i].ymax;
        } else {
            xMin = inData->detections[i].xmin * width;
            yMin = inData->detections[i].ymin * height;
            xMax = inData->detections[i].xmax * width;
            yMax = inData->detections[i].ymax * height;
        }

        float xSize = xMax - xMin;
        float ySize = yMax - yMin;
        float xCenter = xMin + xSize / 2;
        float yCenter = yMin + ySize / 2;

        opDetectionMsg.detections[i].results.resize(1);

        opDetectionMsg.detections[i].id = std::to_string(inData->detections[i].label);
        opDetectionMsg.detections[i].results[0].hypothesis.class_id = inData->detections[i].labelName;
        opDetectionMsg.detections[i].results[0].hypothesis.score = inData->detections[i].confidence;
        opDetectionMsg.detections[i].bbox.center.position.x = xCenter;
        opDetectionMsg.detections[i].bbox.center.position.y = yCenter;
        opDetectionMsg.detections[i].bbox.size_x = xSize;
        opDetectionMsg.detections[i].bbox.size_y = ySize;
    }

    opDetectionMsgs.push_back(opDetectionMsg);
}

Detection2DArrayPtr ImgDetectionConverter::toRosMsgPtr(std::shared_ptr<dai::ImgDetections> inData) {
    std::deque<VisionMsgs::Detection2DArray> msgQueue;
    toRosMsg(inData, msgQueue);
    auto msg = msgQueue.front();
    Detection2DArrayPtr ptr = std::make_shared<VisionMsgs::Detection2DArray>(msg);
    return ptr;
}

}  // namespace depthai_bridge
