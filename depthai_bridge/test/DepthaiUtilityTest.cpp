#include <gtest/gtest.h>

#include "depthai_bridge/depthaiUtility.hpp"

namespace depthai_bridge {

TEST(DepthaiUtilityTest, NormalizesFramePrefix) {
    EXPECT_EQ(normalizeFramePrefix("oak"), "oak");
    EXPECT_EQ(normalizeFramePrefix("/robot1/oak"), "robot1/oak");
    EXPECT_EQ(normalizeFramePrefix("robot1/oak/"), "robot1/oak");
    EXPECT_EQ(normalizeFramePrefix("//robot1/oak//"), "robot1/oak");
    EXPECT_EQ(normalizeFramePrefix("///"), "");
}

TEST(DepthaiUtilityTest, BuildsValidFrameNameFromRosStylePrefix) {
    EXPECT_EQ(getFrameName("/robot1/oak/", "imu_frame"), "robot1/oak_imu_frame");
    EXPECT_EQ(getOpticalFrameName("/robot1/oak/", "rgb"), "robot1/oak_rgb_camera_optical_frame");
}

TEST(DepthaiUtilityTest, ResolvesEmptyNormalizedPrefixToFallback) {
    EXPECT_EQ(resolveFramePrefix("", "oak"), "oak");
    EXPECT_EQ(resolveFramePrefix("/", "oak"), "oak");
    EXPECT_EQ(resolveFramePrefix("///", "/robot1/oak/"), "robot1/oak");
    EXPECT_EQ(resolveFramePrefix("/robot2/oak/", "oak"), "robot2/oak");
}

TEST(DepthaiUtilityTest, BuildsFrameNameWithoutEmptyPrefixSeparator) {
    EXPECT_EQ(getFrameName("", "imu_frame"), "imu_frame");
    EXPECT_EQ(getFrameName("/", "imu_frame"), "imu_frame");
}

}  // namespace depthai_bridge
