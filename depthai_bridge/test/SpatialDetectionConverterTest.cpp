
#include <gtest/gtest.h>

#include <deque>
#include <memory>

#include "depthai_bridge/SpatialDetectionConverter.hpp"

namespace depthai_bridge {

TEST(SpatialDetectionConverterTest, ToRosMsgTest) {
    auto inNetData = std::make_shared<dai::SpatialImgDetections>();
    inNetData->detections.resize(1);
    inNetData->detections[0].xmin = 0.0;
    inNetData->detections[0].ymin = 0.0;
    inNetData->detections[0].xmax = 0.5;
    inNetData->detections[0].ymax = 0.5;
    inNetData->detections[0].labelName = "test";
    inNetData->detections[0].confidence = 0.9;
    inNetData->detections[0].spatialCoordinates.x = 100;
    inNetData->detections[0].spatialCoordinates.y = 200;
    inNetData->detections[0].spatialCoordinates.z = 300;

    dai::ImgTransformation transformation;
    transformation.setSize(640, 480);
    inNetData->transformation = transformation;
    SpatialDetectionConverter converter("testFrame", false, false);

    std::deque<SpatialMessages::SpatialDetectionArray> opDetectionMsgs;
    converter.toRosMsg(inNetData, opDetectionMsgs);

    ASSERT_EQ(opDetectionMsgs.size(), 1);
    ASSERT_EQ(opDetectionMsgs[0].detections.size(), 1);
    EXPECT_EQ(opDetectionMsgs[0].detections[0].results[0].class_id, "test");
    EXPECT_FLOAT_EQ(opDetectionMsgs[0].detections[0].results[0].score, 0.9);
    EXPECT_FLOAT_EQ(opDetectionMsgs[0].detections[0].bbox.center.position.x, 160);
    EXPECT_FLOAT_EQ(opDetectionMsgs[0].detections[0].bbox.center.position.y, 120);
    EXPECT_FLOAT_EQ(opDetectionMsgs[0].detections[0].bbox.size_x, 320);
    EXPECT_FLOAT_EQ(opDetectionMsgs[0].detections[0].bbox.size_y, 240);
    EXPECT_FLOAT_EQ(opDetectionMsgs[0].detections[0].position.x, 0.1);
    EXPECT_FLOAT_EQ(opDetectionMsgs[0].detections[0].position.y, 0.2);
    EXPECT_FLOAT_EQ(opDetectionMsgs[0].detections[0].position.z, 0.3);
}

TEST(SpatialDetectionConverterTest, ToRosVisionMsgTest) {
    auto inNetData = std::make_shared<dai::SpatialImgDetections>();
    inNetData->detections.resize(1);
    inNetData->detections[0].xmin = 0.0;
    inNetData->detections[0].ymin = 0.0;
    inNetData->detections[0].xmax = 0.5;
    inNetData->detections[0].ymax = 0.5;
    inNetData->detections[0].label = 1;
    inNetData->detections[0].labelName = "test";
    inNetData->detections[0].confidence = 0.9;
    inNetData->detections[0].spatialCoordinates.x = 100;
    inNetData->detections[0].spatialCoordinates.y = 200;
    inNetData->detections[0].spatialCoordinates.z = 300;

    dai::ImgTransformation transformation;
    transformation.setSize(640, 480);
    inNetData->transformation = transformation;
    SpatialDetectionConverter converter("testFrame", false, false);

    std::deque<vision_msgs::msg::Detection3DArray> opDetectionMsgs;
    converter.toRosVisionMsg(inNetData, opDetectionMsgs);

    ASSERT_EQ(opDetectionMsgs.size(), 1);
    ASSERT_EQ(opDetectionMsgs[0].detections.size(), 1);
    EXPECT_EQ(opDetectionMsgs[0].detections[0].id, "1");
    EXPECT_EQ(opDetectionMsgs[0].detections[0].results[0].hypothesis.class_id, "test");
    EXPECT_FLOAT_EQ(opDetectionMsgs[0].detections[0].results[0].hypothesis.score, 0.9);
    EXPECT_FLOAT_EQ(opDetectionMsgs[0].detections[0].bbox.center.position.x, 160);
    EXPECT_FLOAT_EQ(opDetectionMsgs[0].detections[0].bbox.center.position.y, 120);
    EXPECT_FLOAT_EQ(opDetectionMsgs[0].detections[0].bbox.size.x, 320);
    EXPECT_FLOAT_EQ(opDetectionMsgs[0].detections[0].bbox.size.y, 240);
    EXPECT_FLOAT_EQ(opDetectionMsgs[0].detections[0].bbox.size.z, 0.01);
    EXPECT_FLOAT_EQ(opDetectionMsgs[0].detections[0].results[0].pose.pose.position.x, 0.1);
    EXPECT_FLOAT_EQ(opDetectionMsgs[0].detections[0].results[0].pose.pose.position.y, 0.2);
    EXPECT_FLOAT_EQ(opDetectionMsgs[0].detections[0].results[0].pose.pose.position.z, 0.3);
}

}  // namespace depthai_bridge
