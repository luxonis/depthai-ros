#include <gtest/gtest.h>

#include <algorithm>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai_bridge/DescriptionRunner.hpp"
#include "depthai_bridge/FrameAliases.hpp"
#include "nlohmann/json.hpp"

TEST(DescriptionRunner, PreservesArgumentsWithoutShellExpansion) {
    const std::string value = "space ' quote $(echo injected); $HOME";
    EXPECT_EQ(depthai_bridge::detail::runDescriptionCommand({"python3", "-c", "import sys; print(sys.argv[1], end='')", value}), value);
}

TEST(DescriptionRunner, RejectsFailuresAndIncludesStderr) {
    try {
        depthai_bridge::detail::runDescriptionCommand({"python3", "-c", "import sys; print('invalid IMU', file=sys.stderr); sys.exit(2)"});
        FAIL() << "Expected nonzero status to throw";
    } catch(const std::runtime_error& error) {
        EXPECT_NE(std::string(error.what()).find("invalid IMU"), std::string::npos);
    }
}

TEST(DescriptionRunner, RejectsEmptyOutput) {
    EXPECT_THROW(depthai_bridge::detail::runDescriptionCommand({"python3", "-c", "pass"}), std::runtime_error);
}

TEST(DescriptionRunner, ResolvesInstalledDescriptionAndDetectedChip) {
    const auto script = ament_index_cpp::get_package_share_directory("depthai_descriptions") + "/scripts/resolve_description.py";
    const auto output = depthai_bridge::detail::runDescriptionCommand(
        {"python3", script, "--output=json", "--detected-imu=BMI270", "camera_model:=OAK-D-SR", "pose_source:=board", "use_base_descr:=true", "has_imu:=true"});
    const auto result = nlohmann::json::parse(output);
    EXPECT_EQ(result.at("model"), "OAK-D-SR");
    EXPECT_EQ(result.at("imu_sensor"), "BMI");
    EXPECT_DOUBLE_EQ(result.at("camera_poses").at("CAM_C").at("xyz").at(1).get<double>(), 0.0);
    EXPECT_NE(result.at("urdf").get<std::string>().find("oak_imu_joint"), std::string::npos);
}

TEST(DescriptionRunner, CustomArgumentsReplaceDriverMappings) {
    const auto share = ament_index_cpp::get_package_share_directory("depthai_descriptions");
    const auto arguments = depthai_bridge::detail::descriptionCommand(
        share, "", "camera_name:=custom", "UNSUPPORTED_IMU", {"camera_model:=UNKNOWN", "base_frame:=driver_override", "has_imu:=true", "pose_source:=board"});
    const auto result = nlohmann::json::parse(depthai_bridge::detail::runDescriptionCommand(arguments));
    const auto xml = result.at("urdf").get<std::string>();
    EXPECT_EQ(result.at("pose_source"), "custom");
    EXPECT_NE(xml.find("custom_center_joint"), std::string::npos);
    EXPECT_EQ(xml.find("driver_override"), std::string::npos);
    EXPECT_EQ(xml.find("imu_joint"), std::string::npos);
    EXPECT_EQ(xml.find("left_camera_joint"), std::string::npos);
}

TEST(DescriptionRunner, DefaultMappingsStillReachCustomDescriptions) {
    const auto arguments = depthai_bridge::detail::descriptionCommand("/share", "/custom path/robot.xacro", "", "BMI270", {"base_frame:=robot"});
    EXPECT_NE(std::find(arguments.begin(), arguments.end(), "--description=/custom path/robot.xacro"), arguments.end());
    EXPECT_NE(std::find(arguments.begin(), arguments.end(), "base_frame:=robot"), arguments.end());
}

TEST(DescriptionRunner, DeprecatedFrameAliases) {
    using dai::CameraBoardSocket;
    using depthai_bridge::detail::legacyCameraFrameAliases;
    const auto sr = legacyCameraFrameAliases("oak", CameraBoardSocket::CAM_A, "OAK-D-SR", false);
    EXPECT_TRUE(sr.empty());  // SR has no ToF sensor.
    const auto tof = legacyCameraFrameAliases("oak", CameraBoardSocket::CAM_A, "OAK-D-SR-POE", false);
    ASSERT_EQ(tof.size(), 2u);
    EXPECT_EQ(tof.front(), std::make_pair(std::string("oak_tof_camera_frame"), std::string("oak_rgb_camera_frame")));
    const auto thermal = legacyCameraFrameAliases("oak", CameraBoardSocket::CAM_E, "OAK-T", false);
    ASSERT_EQ(thermal.size(), 2u);
    EXPECT_EQ(thermal.front(), std::make_pair(std::string("oak_thermal_camera_frame"), std::string("oak_cam_e_camera_frame")));
    const auto rs = legacyCameraFrameAliases("robot/oak", CameraBoardSocket::CAM_A, "OAK-D", true);
    ASSERT_EQ(rs.size(), 1u);
    EXPECT_EQ(rs.front(), std::make_pair(std::string("robot/oak_color_frame"), std::string("robot/oak_color_camera_frame")));
}
