#include <gtest/gtest.h>

#include <deque>
#include <memory>

#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/depthaiUtility.hpp"

namespace depthai_bridge {

TEST(DisparityConverterTest, ConstructorTest) {
    DisparityConverter converter("testFrame", 1.0f, 100.0f, 10.0f, 1000.0f, true);
    EXPECT_EQ(converter.getFocalLength(), 1.0f);
    EXPECT_EQ(converter.getBaseline(), 1.0f);
    EXPECT_EQ(converter.getMinDepth(), 0.1f);
    EXPECT_EQ(converter.getMaxDepth(), 10.0f);
}

TEST(DisparityConverterTest, ToRosMsgTest) {
    DisparityConverter converter("testFrame", 1.0f, 100.0f, 10.0f, 1000.0f, true);
    auto inData = std::make_shared<dai::ImgFrame>();
    inData->setType(dai::ImgFrame::Type::RAW8);
    inData->setData({1, 2, 3, 4});
    inData->setHeight(2);
    inData->setWidth(2);

    std::deque<DisparityMsgs::DisparityImage> outDispImageMsgs;
    converter.toRosMsg(inData, outDispImageMsgs);

    ASSERT_EQ(outDispImageMsgs.size(), 1);
    auto& outDispImageMsg = outDispImageMsgs.front();
    EXPECT_EQ(outDispImageMsg.f, 1.0f);
    EXPECT_EQ(outDispImageMsg.min_disparity, 0.1f);
    EXPECT_EQ(outDispImageMsg.max_disparity, 10.0f);
    EXPECT_EQ(outDispImageMsg.t, 0.01f);
    EXPECT_EQ(outDispImageMsg.image.encoding, sensor_msgs::image_encodings::TYPE_32FC1);
    EXPECT_EQ(outDispImageMsg.image.height, 2);
    EXPECT_EQ(outDispImageMsg.image.width, 2);
    EXPECT_EQ(outDispImageMsg.image.step, 8);
    EXPECT_EQ(outDispImageMsg.image.is_bigendian, true);
    EXPECT_EQ(outDispImageMsg.image.data.size(), 4);
}

TEST(DisparityConverterTest, ToRosMsgPtrTest) {
    DisparityConverter converter("testFrame", 1.0f, 100.0f, 10.0f, 1000.0f, true);
    auto inData = std::make_shared<dai::ImgFrame>();
    inData->setType(dai::ImgFrame::Type::RAW8);
    inData->setData({1, 2, 3, 4});
    inData->setHeight(2);
    inData->setWidth(2);

    auto outDispImagePtr = converter.toRosMsgPtr(inData);

    ASSERT_NE(outDispImagePtr, nullptr);
    EXPECT_EQ(outDispImagePtr->f, 1.0f);
    EXPECT_EQ(outDispImagePtr->min_disparity, 0.1f);
    EXPECT_EQ(outDispImagePtr->max_disparity, 10.0f);
    EXPECT_EQ(outDispImagePtr->t, 0.01f);
    EXPECT_EQ(outDispImagePtr->image.encoding, sensor_msgs::image_encodings::TYPE_32FC1);
    EXPECT_EQ(outDispImagePtr->image.height, 2);
    EXPECT_EQ(outDispImagePtr->image.width, 2);
    EXPECT_EQ(outDispImagePtr->image.step, 8);
    EXPECT_EQ(outDispImagePtr->image.is_bigendian, true);
    EXPECT_EQ(outDispImagePtr->image.data.size(), 4);
}

}  // namespace depthai_bridge
