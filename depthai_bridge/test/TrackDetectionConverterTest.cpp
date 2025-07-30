#include <gtest/gtest.h>

#include "depthai_bridge/TrackDetectionConverter.hpp"
#include "depthai_ros_msgs/msg/track_detection2_d_array.hpp"

namespace depthai_bridge {

TEST(TrackDetectionConverterTest, ToRosMsgTest) {
    TrackDetectionConverter converter("testFrame", 640, 480, false, 0.5, false);

    auto trackData = std::make_shared<dai::Tracklets>();
    dai::Tracklet tracklet;
    tracklet.roi = dai::Rect(0.0, 0.0, 0.5, 0.5);
    tracklet.label = 1;
    tracklet.id = 1;
    tracklet.age = 10;
    tracklet.status = dai::Tracklet::TrackingStatus::TRACKED;
    trackData->tracklets.push_back(tracklet);

    std::deque<depthai_ros_msgs::msg::TrackDetection2DArray> opDetectionMsgs;

    converter.toRosMsg(trackData, opDetectionMsgs);

    ASSERT_EQ(opDetectionMsgs.size(), 1);
    ASSERT_EQ(opDetectionMsgs.front().detections.size(), 1);
    ASSERT_EQ(opDetectionMsgs.front().detections[0].results[0].hypothesis.class_id, "1");
    ASSERT_EQ(opDetectionMsgs.front().detections[0].results[0].hypothesis.score, 0.5);
    ASSERT_EQ(opDetectionMsgs.front().detections[0].bbox.center.position.x, 160);
    ASSERT_EQ(opDetectionMsgs.front().detections[0].bbox.center.position.y, 120);
    ASSERT_EQ(opDetectionMsgs.front().detections[0].bbox.size_x, 320);
    ASSERT_EQ(opDetectionMsgs.front().detections[0].bbox.size_y, 240);
    ASSERT_TRUE(opDetectionMsgs.front().detections[0].is_tracking);
    ASSERT_EQ(opDetectionMsgs.front().detections[0].tracking_id, "1");
    ASSERT_EQ(opDetectionMsgs.front().detections[0].tracking_age, 10);
    ASSERT_EQ(opDetectionMsgs.front().detections[0].tracking_status, static_cast<int32_t>(dai::Tracklet::TrackingStatus::TRACKED));
}

TEST(TrackDetectionConverterTest, ToRosMsgPtrTest) {
    TrackDetectionConverter converter("testFrame", 640, 480, false, 0.5, false);

    auto trackData = std::make_shared<dai::Tracklets>();
    dai::Tracklet tracklet;
    tracklet.roi = dai::Rect(0.0, 0.0, 0.5, 0.5);
    tracklet.label = 1;
    tracklet.id = 1;
    tracklet.age = 10;
    tracklet.status = dai::Tracklet::TrackingStatus::TRACKED;
    trackData->tracklets.push_back(tracklet);

    auto msgPtr = converter.toRosMsgPtr(trackData);

    ASSERT_EQ(msgPtr->detections.size(), 1);
    ASSERT_EQ(msgPtr->detections[0].results[0].hypothesis.class_id, "1");
    ASSERT_EQ(msgPtr->detections[0].results[0].hypothesis.score, 0.5);
    ASSERT_EQ(msgPtr->detections[0].bbox.center.position.x, 160);
    ASSERT_EQ(msgPtr->detections[0].bbox.center.position.y, 120);
    ASSERT_EQ(msgPtr->detections[0].bbox.size_x, 320);
    ASSERT_EQ(msgPtr->detections[0].bbox.size_y, 240);
    ASSERT_TRUE(msgPtr->detections[0].is_tracking);
    ASSERT_EQ(msgPtr->detections[0].tracking_id, "1");
    ASSERT_EQ(msgPtr->detections[0].tracking_age, 10);
    ASSERT_EQ(msgPtr->detections[0].tracking_status, static_cast<int32_t>(dai::Tracklet::TrackingStatus::TRACKED));
}

}  // namespace depthai_bridge
