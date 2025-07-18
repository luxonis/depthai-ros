#include <gtest/gtest.h>

#include <depthai/common/Point3fRGBA.hpp>
#include <deque>
#include <memory>

#include "depthai_bridge/PointCloudConverter.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

TEST(PointCloudConverterTest, ToRosMsgColoredPointsTest) {
    dai::PointCloudData pclData;
    pclData.setWidth(2);
    pclData.setHeight(1);
    pclData.setColor(true);
    pclData.setSparse(false);

    std::vector<dai::Point3fRGBA> pointsRGB = {{1.0f, 2.0f, 3.0f, 255, 0, 0}, {4.0f, 5.0f, 6.0f, 0, 255, 0}};
    pclData.setPointsRGB(pointsRGB);

    depthai_bridge::PointCloudConverter converter("test_frame", false);

    std::deque<sensor_msgs::msg::PointCloud2> pclMsgs;
    converter.toRosMsg(std::make_shared<dai::PointCloudData>(pclData), pclMsgs);

    ASSERT_EQ(pclMsgs.size(), 1);
    const auto& msg = pclMsgs.front();

    EXPECT_EQ(msg.header.frame_id, "test_frame");

    ASSERT_EQ(msg.fields.size(), 4);
    EXPECT_EQ(msg.fields[0].name, "x");
    EXPECT_EQ(msg.fields[1].name, "y");
    EXPECT_EQ(msg.fields[2].name, "z");
    EXPECT_EQ(msg.fields[3].name, "rgb");

    ASSERT_EQ(msg.data.size(), 32);  // 2x16
    const float* data = reinterpret_cast<const float*>(msg.data.data());
    EXPECT_FLOAT_EQ(data[0], 1.0f);
    EXPECT_FLOAT_EQ(data[1], 2.0f);
    EXPECT_FLOAT_EQ(data[2], 3.0f);
    uint32_t rgb;
    std::memcpy(&rgb, &data[3], sizeof(float));
    EXPECT_EQ(rgb, 0xFF0000);  // Red color

    EXPECT_FLOAT_EQ(data[4], 4.0f);
    EXPECT_FLOAT_EQ(data[5], 5.0f);
    EXPECT_FLOAT_EQ(data[6], 6.0f);
    std::memcpy(&rgb, &data[7], sizeof(float));
    EXPECT_EQ(rgb, 0x00FF00);  // Green color
}

TEST(PointCloudConverterTest, ToRosMsgNonColoredPointsTest) {
    dai::PointCloudData pclData;
    pclData.setWidth(2);
    pclData.setHeight(1);
    pclData.setColor(false);
    pclData.setSparse(false);

    std::vector<dai::Point3f> points = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}};
    pclData.setPoints(points);

    depthai_bridge::PointCloudConverter converter("test_frame", false);

    std::deque<sensor_msgs::msg::PointCloud2> pclMsgs;
    converter.toRosMsg(std::make_shared<dai::PointCloudData>(pclData), pclMsgs);

    ASSERT_EQ(pclMsgs.size(), 1);
    const auto& msg = pclMsgs.front();

    EXPECT_EQ(msg.header.frame_id, "test_frame");

    ASSERT_EQ(msg.fields.size(), 3);
    EXPECT_EQ(msg.fields[0].name, "x");
    EXPECT_EQ(msg.fields[1].name, "y");
    EXPECT_EQ(msg.fields[2].name, "z");

    ASSERT_EQ(msg.data.size(), 24);  // 2x12
    const float* data = reinterpret_cast<const float*>(msg.data.data());
    EXPECT_FLOAT_EQ(data[0], 1.0f);
    EXPECT_FLOAT_EQ(data[1], 2.0f);
    EXPECT_FLOAT_EQ(data[2], 3.0f);

    EXPECT_FLOAT_EQ(data[3], 4.0f);
    EXPECT_FLOAT_EQ(data[4], 5.0f);
    EXPECT_FLOAT_EQ(data[5], 6.0f);
}
