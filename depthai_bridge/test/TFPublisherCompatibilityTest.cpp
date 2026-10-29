#include <gtest/gtest.h>

#include <cstdlib>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "rclcpp/rclcpp.hpp"

std::size_t developTFPublisherSize();
std::string developTFPublisherClient(std::shared_ptr<rclcpp::Node>, const dai::CalibrationHandler&, const std::string&);

TEST(TFPublisherCompatibility, DevelopClientLayoutAndDefaultModelPolicy) {
    ASSERT_EQ(sizeof(depthai_bridge::TFPublisher), developTFPublisherSize());
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    {
        auto node = std::make_shared<rclcpp::Node>("oak", rclcpp::NodeOptions().context(context));
        dai::CalibrationHandler calibration(ament_index_cpp::get_package_share_directory("depthai_bridge") + "/resources/cal.json");
        // A runtime virtualenv must not replace the ROS/Xacro interpreter used
        // when the package was built (develop's xacro executable has a shebang).
        const char* path = std::getenv("PATH");
        const std::string originalPath = path ? path : "";
        struct RestorePath {
            std::string value;
            ~RestorePath() {
                setenv("PATH", value.c_str(), 1);
            }
        } restore{originalPath};
        setenv("PATH", "/nonexistent-python-environment", 1);
        for(const auto& model : {"OAK-D-S2", "UNKNOWN", "PERCEPT"}) {
            const auto xml = developTFPublisherClient(node, calibration, model);
            const auto expectedMesh = std::string(model) == "OAK-D-S2" ? "OAK-D-PRO.stl" : "OAK-D-S2.stl";
            EXPECT_NE(xml.find(expectedMesh), std::string::npos) << model;
            EXPECT_EQ(xml.find("oak_imu_joint"), std::string::npos) << model;
        }
    }
    context->shutdown("test complete");
}
