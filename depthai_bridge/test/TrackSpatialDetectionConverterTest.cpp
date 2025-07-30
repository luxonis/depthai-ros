#include <gtest/gtest.h>

#include <memory>

#include "depthai_bridge/TrackSpatialDetectionConverter.hpp"
#include "depthai_ros_msgs/msg/track_detection2_d_array.hpp"

namespace depthai_bridge {

TEST(TrackSpatialDetectionConverterTest, ToRosMsgNormalized) {
    TrackSpatialDetectionConverter converter("test_frame", 640, 480, true, 0.5, false);

    auto trackData = std::make_shared<dai::Tracklets>();
    dai::Tracklet tracklet;
    tracklet.roi = dai::Rect(0.0, 0.0, 0.5, 0.5);
    tracklet.label = 1;
    tracklet.id = 1;
    tracklet.age = 10;
    tracklet.status = dai::Tracklet::TrackingStatus::TRACKED;
    tracklet.spatialCoordinates.x = 100;
    tracklet.spatialCoordinates.y = 200;
    tracklet.spatialCoordinates.z = 300;
    trackData->tracklets.push_back(tracklet);

    std::deque<depthai_ros_msgs::msg::TrackDetection2DArray> opDetectionMsgs;
    converter.toRosMsg(trackData, opDetectionMsgs);

    ASSERT_EQ(opDetectionMsgs.size(), 1);
    auto& msg = opDetectionMsgs.front();
    ASSERT_EQ(msg.detections.size(), 1);
    auto& detection = msg.detections[0];
    EXPECT_EQ(detection.results[0].hypothesis.class_id, "1");
    EXPECT_FLOAT_EQ(detection.results[0].hypothesis.score, 0.5);
    EXPECT_FLOAT_EQ(detection.bbox.center.position.x, 0.25);
    EXPECT_FLOAT_EQ(detection.bbox.center.position.y, 0.25);
    EXPECT_FLOAT_EQ(detection.bbox.size_x, 0.5);
    EXPECT_FLOAT_EQ(detection.bbox.size_y, 0.5);
    EXPECT_TRUE(detection.is_tracking);
    EXPECT_EQ(detection.tracking_id, "1");
    EXPECT_EQ(detection.tracking_age, 10);
    EXPECT_EQ(detection.tracking_status, static_cast<int32_t>(dai::Tracklet::TrackingStatus::TRACKED));
    EXPECT_FLOAT_EQ(detection.results[0].pose.pose.position.x, 0.1);
    EXPECT_FLOAT_EQ(detection.results[0].pose.pose.position.y, 0.2);
    EXPECT_FLOAT_EQ(detection.results[0].pose.pose.position.z, 0.3);
}

TEST(TrackSpatialDetectionConverterTest, ToRosMsgDenormalized) {
    TrackSpatialDetectionConverter converter("test_frame", 640, 480, false, 0.5, false);

    auto trackData = std::make_shared<dai::Tracklets>();
    dai::Tracklet tracklet;
    tracklet.roi = dai::Rect(0.0, 0.0, 0.5, 0.5);
    tracklet.label = 1;
    tracklet.id = 1;
    tracklet.age = 10;
    tracklet.status = dai::Tracklet::TrackingStatus::TRACKED;
    tracklet.spatialCoordinates.x = 100;
    tracklet.spatialCoordinates.y = 200;
    tracklet.spatialCoordinates.z = 300;
    trackData->tracklets.push_back(tracklet);

    std::deque<depthai_ros_msgs::msg::TrackDetection2DArray> opDetectionMsgs;
    converter.toRosMsg(trackData, opDetectionMsgs);

    ASSERT_EQ(opDetectionMsgs.size(), 1);
    auto& msg = opDetectionMsgs.front();
    ASSERT_EQ(msg.detections.size(), 1);
    auto& detection = msg.detections[0];
    EXPECT_EQ(detection.results[0].hypothesis.class_id, "1");
    EXPECT_FLOAT_EQ(detection.results[0].hypothesis.score, 0.5);
    EXPECT_FLOAT_EQ(detection.bbox.center.position.x, 160);
    EXPECT_FLOAT_EQ(detection.bbox.center.position.y, 120);
    EXPECT_FLOAT_EQ(detection.bbox.size_x, 320);
    EXPECT_FLOAT_EQ(detection.bbox.size_y, 240);
    EXPECT_TRUE(detection.is_tracking);
    EXPECT_EQ(detection.tracking_id, "1");
    EXPECT_EQ(detection.tracking_age, 10);
    EXPECT_EQ(detection.tracking_status, static_cast<int32_t>(dai::Tracklet::TrackingStatus::TRACKED));
    EXPECT_FLOAT_EQ(detection.results[0].pose.pose.position.x, 0.1);
    EXPECT_FLOAT_EQ(detection.results[0].pose.pose.position.y, 0.2);
    EXPECT_FLOAT_EQ(detection.results[0].pose.pose.position.z, 0.3);
}

TEST(TrackSpatialDetectionConverterTest, ToRosMsgPtr) {
    TrackSpatialDetectionConverter converter("test_frame", 640, 480, false, 0.5, false);

    auto trackData = std::make_shared<dai::Tracklets>();
    dai::Tracklet tracklet;
    tracklet.roi = dai::Rect(0.0, 0.0, 0.5, 0.5);
    tracklet.label = 1;
    tracklet.id = 1;
    tracklet.age = 10;
    tracklet.status = dai::Tracklet::TrackingStatus::TRACKED;
    tracklet.spatialCoordinates.x = 100;
    tracklet.spatialCoordinates.y = 200;
    tracklet.spatialCoordinates.z = 300;
    trackData->tracklets.push_back(tracklet);

    auto msgPtr = converter.toRosMsgPtr(trackData);

    ASSERT_NE(msgPtr, nullptr);
    ASSERT_EQ(msgPtr->detections.size(), 1);
    auto& detection = msgPtr->detections[0];
    EXPECT_EQ(detection.results[0].hypothesis.class_id, "1");
    EXPECT_FLOAT_EQ(detection.results[0].hypothesis.score, 0.5);
    EXPECT_FLOAT_EQ(detection.bbox.center.position.x, 160);
    EXPECT_FLOAT_EQ(detection.bbox.center.position.y, 120);
    EXPECT_FLOAT_EQ(detection.bbox.size_x, 320);
    EXPECT_FLOAT_EQ(detection.bbox.size_y, 240);
    EXPECT_TRUE(detection.is_tracking);
    EXPECT_EQ(detection.tracking_id, "1");
    EXPECT_EQ(detection.tracking_age, 10);
    EXPECT_EQ(detection.tracking_status, static_cast<int32_t>(dai::Tracklet::TrackingStatus::TRACKED));
    EXPECT_FLOAT_EQ(detection.results[0].pose.pose.position.x, 0.1);
    EXPECT_FLOAT_EQ(detection.results[0].pose.pose.position.y, 0.2);
    EXPECT_FLOAT_EQ(detection.results[0].pose.pose.position.z, 0.3);
}

}  // namespace depthai_bridge
