#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <memory>

#include "depthai/pipeline/datatype/Odometry.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai_bridge/TransformDataConverter.hpp"

namespace depthai_bridge {

TEST(TransformDataConverterTest, ToRosMsgOdometryTest) {
    auto inData = std::make_shared<dai::TransformData>(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0);

    TransformDataConverter converter("test_frame", "child_frame", false);
    std::vector<double> covariance = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1};
    converter.setCovariance(covariance);

    std::deque<nav_msgs::msg::Odometry> odomMsgs;
    converter.toRosMsg(inData, odomMsgs);

    ASSERT_EQ(odomMsgs.size(), 1);
    auto& odomMsg = odomMsgs.front();
    ASSERT_EQ(odomMsg.header.frame_id, "test_frame");
    ASSERT_EQ(odomMsg.child_frame_id, "child_frame");
    ASSERT_FLOAT_EQ(odomMsg.pose.pose.position.x, 1.0);
    ASSERT_FLOAT_EQ(odomMsg.pose.pose.position.y, 2.0);
    ASSERT_FLOAT_EQ(odomMsg.pose.pose.position.z, 3.0);
    ASSERT_FLOAT_EQ(odomMsg.pose.pose.orientation.x, 0.0);
    ASSERT_FLOAT_EQ(odomMsg.pose.pose.orientation.y, 0.0);
    ASSERT_FLOAT_EQ(odomMsg.pose.pose.orientation.z, 0.0);
    ASSERT_FLOAT_EQ(odomMsg.pose.pose.orientation.w, 1.0);
    ASSERT_DOUBLE_EQ(odomMsg.twist.twist.linear.x, 0.0);
    ASSERT_DOUBLE_EQ(odomMsg.twist.twist.linear.y, 0.0);
    ASSERT_DOUBLE_EQ(odomMsg.twist.twist.linear.z, 0.0);
    for(int i = 0; i < 36; ++i) {
        ASSERT_FLOAT_EQ(odomMsg.pose.covariance[i], covariance[i]);
    }
}

TEST(TransformDataConverterTest, OdometryLinearVelocityTest) {
    auto inData = std::make_shared<dai::Odometry>(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0, dai::Point3d{4.0, -5.0, 6.0});
    // Preserve velocity even when the message is passed through the TransformData API.
    std::shared_ptr<dai::TransformData> transform = inData;
    TransformDataConverter converter("odom", "base_link");
    std::deque<nav_msgs::msg::Odometry> odomMsgs;
    converter.toRosMsg(transform, odomMsgs);

    ASSERT_EQ(odomMsgs.size(), 1);
    const auto& msg = odomMsgs.front();
    EXPECT_EQ(msg.header.frame_id, "odom");
    EXPECT_EQ(msg.child_frame_id, "base_link");
    EXPECT_DOUBLE_EQ(msg.pose.pose.position.x, 1.0);
    EXPECT_DOUBLE_EQ(msg.pose.pose.position.y, 2.0);
    EXPECT_DOUBLE_EQ(msg.pose.pose.position.z, 3.0);
    EXPECT_DOUBLE_EQ(msg.twist.twist.linear.x, 4.0);
    EXPECT_DOUBLE_EQ(msg.twist.twist.linear.y, -5.0);
    EXPECT_DOUBLE_EQ(msg.twist.twist.linear.z, 6.0);
    EXPECT_DOUBLE_EQ(msg.twist.twist.angular.x, 0.0);
    EXPECT_DOUBLE_EQ(msg.twist.twist.angular.y, 0.0);
    EXPECT_DOUBLE_EQ(msg.twist.twist.angular.z, 0.0);
}

TEST(TransformDataConverterTest, OdometryVelocityInChildFrameTest) {
    // A body rotated 90 degrees about Z sees world +X as body -Y.
    const double halfSqrt2 = std::sqrt(0.5);
    auto inData = std::make_shared<dai::Odometry>(1.0, 2.0, 3.0, 0.0, 0.0, halfSqrt2, halfSqrt2, dai::Point3d{4.0, 5.0, 6.0});
    TransformDataConverter converter("odom", "base_link");
    std::deque<nav_msgs::msg::Odometry> odomMsgs;
    converter.toRosMsg(inData, odomMsgs);

    ASSERT_EQ(odomMsgs.size(), 1);
    const auto& msg = odomMsgs.front();
    EXPECT_NEAR(msg.twist.twist.linear.x, 5.0, 1e-12);
    EXPECT_NEAR(msg.twist.twist.linear.y, -4.0, 1e-12);
    EXPECT_NEAR(msg.twist.twist.linear.z, 6.0, 1e-12);
    EXPECT_DOUBLE_EQ(msg.pose.pose.orientation.z, halfSqrt2);
    EXPECT_DOUBLE_EQ(msg.pose.pose.orientation.w, halfSqrt2);
}

TEST(TransformDataConverterTest, ToRosMsgTransformStampedTest) {
    auto inData = std::make_shared<dai::TransformData>(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0);

    TransformDataConverter converter("test_frame", "child_frame", false);

    std::deque<geometry_msgs::msg::TransformStamped> transformMsgs;
    converter.toRosMsg(inData, transformMsgs);

    ASSERT_EQ(transformMsgs.size(), 1);
    auto& transformMsg = transformMsgs.front();
    ASSERT_EQ(transformMsg.header.frame_id, "test_frame");
    ASSERT_EQ(transformMsg.child_frame_id, "child_frame");
    ASSERT_FLOAT_EQ(transformMsg.transform.translation.x, 1.0);
    ASSERT_FLOAT_EQ(transformMsg.transform.translation.y, 2.0);
    ASSERT_FLOAT_EQ(transformMsg.transform.translation.z, 3.0);
    ASSERT_FLOAT_EQ(transformMsg.transform.rotation.x, 0.0);
    ASSERT_FLOAT_EQ(transformMsg.transform.rotation.y, 0.0);
    ASSERT_FLOAT_EQ(transformMsg.transform.rotation.z, 0.0);
    ASSERT_FLOAT_EQ(transformMsg.transform.rotation.w, 1.0);
}

TEST(TransformDataConverterTest, ToRosMsgPoseWithCovarianceStampedTest) {
    auto inData = std::make_shared<dai::TransformData>(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0);

    TransformDataConverter converter("test_frame", "child_frame", false);
    std::array<double, 36> covariance = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1};
    converter.setCovariance(covariance);

    std::deque<geometry_msgs::msg::PoseWithCovarianceStamped> poseMsgs;
    converter.toRosMsg(inData, poseMsgs);

    ASSERT_EQ(poseMsgs.size(), 1);
    auto& poseMsg = poseMsgs.front();
    ASSERT_EQ(poseMsg.header.frame_id, "test_frame");
    ASSERT_FLOAT_EQ(poseMsg.pose.pose.position.x, 1.0);
    ASSERT_FLOAT_EQ(poseMsg.pose.pose.position.y, 2.0);
    ASSERT_FLOAT_EQ(poseMsg.pose.pose.position.z, 3.0);
    ASSERT_FLOAT_EQ(poseMsg.pose.pose.orientation.x, 0.0);
    ASSERT_FLOAT_EQ(poseMsg.pose.pose.orientation.y, 0.0);
    ASSERT_FLOAT_EQ(poseMsg.pose.pose.orientation.z, 0.0);
    ASSERT_FLOAT_EQ(poseMsg.pose.pose.orientation.w, 1.0);
    for(int i = 0; i < 36; ++i) {
        ASSERT_FLOAT_EQ(poseMsg.pose.covariance[i], covariance[i]);
    }
}

TEST(TransformDataConverterTest, FixQuaternionTest) {
    auto inData = std::make_shared<dai::TransformData>(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0);

    TransformDataConverter converter("test_frame", "child_frame", false);
    converter.fixQuaternion();

    std::deque<nav_msgs::msg::Odometry> odomMsgs;
    converter.toRosMsg(inData, odomMsgs);

    ASSERT_EQ(odomMsgs.size(), 1);
    auto& odomMsg = odomMsgs.front();
    ASSERT_FLOAT_EQ(odomMsg.pose.pose.orientation.x, 0.0);
    ASSERT_FLOAT_EQ(odomMsg.pose.pose.orientation.y, 0.0);
    ASSERT_FLOAT_EQ(odomMsg.pose.pose.orientation.z, 0.0);
    ASSERT_FLOAT_EQ(odomMsg.pose.pose.orientation.w, 1.0);
}

}  // namespace depthai_bridge
