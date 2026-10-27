#include <gtest/gtest.h>

#include <depthai/common/ImgTransformations.hpp>
#include <deque>
#include <memory>

#include "depthai_bridge/ImgDetectionConverter.hpp"

namespace depthai_bridge {

TEST(ImgDetectionConverterTest, ToRosMsgTest) {
    auto inData = std::make_shared<dai::ImgDetections>();
    inData->detections.resize(1);
    inData->detections[0].xmin = 0.0;
    inData->detections[0].ymin = 0.0;
    inData->detections[0].xmax = 0.5;
    inData->detections[0].ymax = 0.5;
    inData->detections[0].label = 1;
    inData->detections[0].labelName = "test";
    inData->detections[0].confidence = 0.9;

    dai::ImgTransformation transformation;
    transformation.setSize(640, 480);
    inData->transformation = transformation;

    ImgDetectionConverter converter("test_frame", false, false);

    std::deque<VisionMsgs::Detection2DArray> opDetectionMsgs;
    converter.toRosMsg(inData, opDetectionMsgs);

    ASSERT_EQ(opDetectionMsgs.size(), 1);
    auto& detectionMsg = opDetectionMsgs.front();
    ASSERT_EQ(detectionMsg.detections.size(), 1);
    ASSERT_EQ(detectionMsg.detections[0].id, "1");
    ASSERT_EQ(detectionMsg.detections[0].results[0].hypothesis.class_id, "test");
    ASSERT_FLOAT_EQ(detectionMsg.detections[0].results[0].hypothesis.score, 0.9);
    ASSERT_FLOAT_EQ(detectionMsg.detections[0].bbox.center.position.x, 160);
    ASSERT_FLOAT_EQ(detectionMsg.detections[0].bbox.center.position.y, 120);
    ASSERT_FLOAT_EQ(detectionMsg.detections[0].bbox.size_x, 320);
    ASSERT_FLOAT_EQ(detectionMsg.detections[0].bbox.size_y, 240);
}

TEST(ImgDetectionConverterTest, ToRosMsgPtrTest) {
    auto inData = std::make_shared<dai::ImgDetections>();
    inData->detections.resize(1);
    inData->detections[0].xmin = 0.0;
    inData->detections[0].ymin = 0.0;
    inData->detections[0].xmax = 0.5;
    inData->detections[0].ymax = 0.5;
    inData->detections[0].label = 1;
    inData->detections[0].labelName = "test";
    inData->detections[0].confidence = 0.9;

    dai::ImgTransformation transformation;
    transformation.setSize(640, 480);
    inData->transformation = transformation;

    ImgDetectionConverter converter("test_frame", false, false);

    auto msgPtr = converter.toRosMsgPtr(inData);

    ASSERT_NE(msgPtr, nullptr);
    ASSERT_EQ(msgPtr->detections.size(), 1);
    ASSERT_EQ(msgPtr->detections[0].id, "1");
    ASSERT_EQ(msgPtr->detections[0].results[0].hypothesis.class_id, "test");
    ASSERT_FLOAT_EQ(msgPtr->detections[0].results[0].hypothesis.score, 0.9);
    ASSERT_FLOAT_EQ(msgPtr->detections[0].bbox.center.position.x, 160);
    ASSERT_FLOAT_EQ(msgPtr->detections[0].bbox.center.position.y, 120);
    ASSERT_FLOAT_EQ(msgPtr->detections[0].bbox.size_x, 320);
    ASSERT_FLOAT_EQ(msgPtr->detections[0].bbox.size_y, 240);
}

}  // namespace depthai_bridge
