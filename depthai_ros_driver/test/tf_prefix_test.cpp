#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace {
class TestNode : public BaseNode {
   public:
    explicit TestNode(const std::shared_ptr<rclcpp::Node>& node) : BaseNode("imu", node, nullptr, "OAK-D", false) {}

    void setupQueues(std::shared_ptr<dai::Device>) override {}
    void setNames() override {}
    void setInOut(std::shared_ptr<dai::Pipeline>) override {}
    void closeQueues() override {}
};

class TFPrefixTest : public ::testing::Test {
   protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }
};

TEST_F(TFPrefixTest, UsesConfiguredBaseFrameWhenPublishingCalibrationTF) {
    auto node = std::make_shared<rclcpp::Node>("oak");
    node->declare_parameter("driver.i_publish_tf_from_calibration", true);
    node->declare_parameter<std::string>("driver.i_tf_base_frame", "robot1/oakd");
    TestNode testNode(node);

    EXPECT_EQ(testNode.getFrameName("imu"), "robot1/oakd_imu");
    EXPECT_EQ(testNode.getOpticalFrameName("rgb"), "robot1/oakd_rgb_camera_optical_frame");
}

TEST_F(TFPrefixTest, FallsBackToNodeNameWhenCalibrationTFIsDisabled) {
    auto node = std::make_shared<rclcpp::Node>("oak");
    node->declare_parameter("driver.i_publish_tf_from_calibration", false);
    node->declare_parameter<std::string>("driver.i_tf_base_frame", "robot1/oakd");
    TestNode testNode(node);

    EXPECT_EQ(testNode.getFrameName("imu"), "oak_imu");
    EXPECT_EQ(testNode.getOpticalFrameName("rgb"), "oak_rgb_camera_optical_frame");
}
}  // namespace
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
