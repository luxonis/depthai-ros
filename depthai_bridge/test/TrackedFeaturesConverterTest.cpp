#include <gtest/gtest.h>

#include <deque>

#include "depthai_bridge/TrackedFeaturesConverter.hpp"
#include "depthai_ros_msgs/msg/tracked_feature.hpp"
#include "depthai_ros_msgs/msg/tracked_features.hpp"

namespace depthai_bridge {

TEST(TrackedFeaturesConverterTest, ToRosMsgTest) {
    TrackedFeaturesConverter converter("test_frame");
    auto inFeatures = std::make_shared<dai::TrackedFeatures>();
    dai::TrackedFeature feature;
    feature.position.x = 1.0f;
    feature.position.y = 2.0f;
    feature.age = 3;
    feature.id = 4;
    feature.harrisScore = 5.0f;
    feature.trackingError = 6.0f;
    inFeatures->trackedFeatures.push_back(feature);

    std::deque<depthai_ros_msgs::msg::TrackedFeatures> featureMsgs;
    converter.toRosMsg(inFeatures, featureMsgs);

    // Verify the output
    ASSERT_EQ(featureMsgs.size(), 1);
    const auto& msg = featureMsgs.front();
    ASSERT_EQ(msg.features.size(), 1);
    const auto& ft = msg.features.front();
    EXPECT_FLOAT_EQ(ft.position.x, 1.0f);
    EXPECT_FLOAT_EQ(ft.position.y, 2.0f);
    EXPECT_EQ(ft.age, 3);
    EXPECT_EQ(ft.id, 4);
    EXPECT_FLOAT_EQ(ft.harris_score, 5.0f);
    EXPECT_FLOAT_EQ(ft.tracking_error, 6.0f);
}
}  // namespace depthai_bridge
