#include "depthai_bridge/SpatialDetectionConverter.hpp"

#include "depthai_bridge/depthaiUtility.hpp"

namespace depthai_bridge {

SpatialDetectionConverter::SpatialDetectionConverter(std::string frameName, bool normalized, bool getBaseDeviceTimestamp)
    : BaseConverter(std::move(frameName), getBaseDeviceTimestamp), normalized(normalized) {}

SpatialDetectionConverter::~SpatialDetectionConverter() = default;

void SpatialDetectionConverter::toRosMsg(std::shared_ptr<dai::SpatialImgDetections> inNetData,
                                         std::deque<SpatialMessages::SpatialDetectionArray>& opDetectionMsgs) {
    SpatialMessages::SpatialDetectionArray opDetectionMsg;

    opDetectionMsg.header = getRosHeader(inNetData);
    opDetectionMsg.detections.resize(inNetData->detections.size());

    auto [width, height] = inNetData->transformation->getSize();
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

        opDetectionMsg.detections[i].results[0].class_id = inNetData->detections[i].labelName;
        opDetectionMsg.detections[i].results[0].score = inNetData->detections[i].confidence;

        opDetectionMsg.detections[i].bbox.center.position.x = xCenter;
        opDetectionMsg.detections[i].bbox.center.position.y = yCenter;
        opDetectionMsg.detections[i].bbox.size_x = xSize;
        opDetectionMsg.detections[i].bbox.size_y = ySize;

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

void SpatialDetectionConverter::toRosVisionMsg(std::shared_ptr<dai::SpatialImgDetections> inNetData,
                                               std::deque<vision_msgs::msg::Detection3DArray>& opDetectionMsgs) {
    vision_msgs::msg::Detection3DArray opDetectionMsg;

    opDetectionMsg.header = getRosHeader(inNetData);
    opDetectionMsg.detections.resize(inNetData->detections.size());

    auto [width, height] = inNetData->transformation->getSize();
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
        opDetectionMsg.detections[i].results[0].hypothesis.class_id = inNetData->detections[i].labelName;
        opDetectionMsg.detections[i].results[0].hypothesis.score = inNetData->detections[i].confidence;
        opDetectionMsg.detections[i].bbox.center.position.x = xCenter;
        opDetectionMsg.detections[i].bbox.center.position.y = yCenter;
        opDetectionMsg.detections[i].bbox.size.x = xSize;
        opDetectionMsg.detections[i].bbox.size.y = ySize;
        opDetectionMsg.detections[i].bbox.size.z = 0.01;

        // converting mm to meters since per ros rep-103 lenght should always be in meters
        opDetectionMsg.detections[i].results[0].pose.pose.position.x = inNetData->detections[i].spatialCoordinates.x / 1000;
        opDetectionMsg.detections[i].results[0].pose.pose.position.y = inNetData->detections[i].spatialCoordinates.y / 1000;
        opDetectionMsg.detections[i].results[0].pose.pose.position.z = inNetData->detections[i].spatialCoordinates.z / 1000;
    }

    opDetectionMsgs.push_back(opDetectionMsg);
}

}  // namespace depthai_bridge
