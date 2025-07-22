#include <gtest/gtest.h>

#include <depthai/common/Point3fRGBA.hpp>
#include <deque>
#include <memory>

#include "depthai_bridge/PointCloudConverter.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace depthai_bridge {

TEST(PointCloudConverterTest, ToRosMsgColoredPointsTest) {
    dai::PointCloudData pclData;
    pclData.setWidth(2);
    pclData.setHeight(1);
    pclData.setColor(true);
    pclData.setSparse(false);

    std::vector<dai::Point3fRGBA> pointsRGB = {{1.0f, 2.0f, 3.0f, 255, 0, 0}, {4.0f, 5.0f, 6.0f, 0, 255, 0}};
    pclData.setPointsRGB(pointsRGB);

    PointCloudConverter converter("test_frame", false);

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

    PointCloudConverter converter("test_frame", false);

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
TEST(PointCloudConverterTest, TestDepthUnits) {
    PointCloudConverter converter("test_frame", false);

    // Test MILLIMETER
    converter.setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::MILLIMETER);
    EXPECT_FLOAT_EQ(converter.getScaleFactor(), 1.0f);

    // Test METER
    converter.setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::METER);
    EXPECT_FLOAT_EQ(converter.getScaleFactor(), 0.001f);

    // Test CENTIMETER
    converter.setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::CENTIMETER);
    EXPECT_FLOAT_EQ(converter.getScaleFactor(), 0.01f);

    // Test FOOT
    converter.setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::FOOT);
    EXPECT_FLOAT_EQ(converter.getScaleFactor(), 0.3048f);

    // Test INCH
    converter.setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::INCH);
    EXPECT_FLOAT_EQ(converter.getScaleFactor(), 0.0254f);

    // Test CUSTOM
    converter.setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::CUSTOM);
    EXPECT_FLOAT_EQ(converter.getScaleFactor(), 1.0f);
}
TEST(PointCloudConverterTest, ToRosMsgColoredPointsWithDepthUnitsTest) {
    dai::PointCloudData pclData;
    pclData.setWidth(2);
    pclData.setHeight(1);
    pclData.setColor(true);
    pclData.setSparse(false);

    std::vector<dai::Point3fRGBA> pointsRGB = {{1.0f, 2.0f, 3.0f, 255, 0, 0}, {4.0f, 5.0f, 6.0f, 0, 255, 0}};
    pclData.setPointsRGB(pointsRGB);

    PointCloudConverter converter("test_frame", false);

    // Test MILLIMETER
    converter.setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::MILLIMETER);
    std::deque<sensor_msgs::msg::PointCloud2> pclMsgs;
    converter.toRosMsg(std::make_shared<dai::PointCloudData>(pclData), pclMsgs);
    ASSERT_EQ(pclMsgs.size(), 1);
    const auto& msg = pclMsgs.front();
    const float* data = reinterpret_cast<const float*>(msg.data.data());
    EXPECT_NEAR(data[0], 1.0f, 1e-6);
    EXPECT_NEAR(data[1], 2.0f, 1e-6);
    EXPECT_NEAR(data[2], 3.0f, 1e-6);
    EXPECT_NEAR(data[4], 4.0f, 1e-6);
    EXPECT_NEAR(data[5], 5.0f, 1e-6);
    EXPECT_NEAR(data[6], 6.0f, 1e-6);

    // Test METER
    converter.setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::METER);
    pclMsgs.clear();
    converter.toRosMsg(std::make_shared<dai::PointCloudData>(pclData), pclMsgs);
    ASSERT_EQ(pclMsgs.size(), 1);
    const auto& msgMeter = pclMsgs.front();
    const float* dataMeter = reinterpret_cast<const float*>(msgMeter.data.data());
    EXPECT_NEAR(dataMeter[0], 0.001f, 1e-6);
    EXPECT_NEAR(dataMeter[1], 0.002f, 1e-6);
    EXPECT_NEAR(dataMeter[2], 0.003f, 1e-6);
    EXPECT_NEAR(dataMeter[4], 0.004f, 1e-6);
    EXPECT_NEAR(dataMeter[5], 0.005f, 1e-6);
    EXPECT_NEAR(dataMeter[6], 0.006f, 1e-6);

    // Test CENTIMETER
    converter.setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::CENTIMETER);
    pclMsgs.clear();
    converter.toRosMsg(std::make_shared<dai::PointCloudData>(pclData), pclMsgs);
    ASSERT_EQ(pclMsgs.size(), 1);
    const auto& msgCentimeter = pclMsgs.front();
    const float* dataCentimeter = reinterpret_cast<const float*>(msgCentimeter.data.data());
    EXPECT_NEAR(dataCentimeter[0], 0.01f, 1e-6);
    EXPECT_NEAR(dataCentimeter[1], 0.02f, 1e-6);
    EXPECT_NEAR(dataCentimeter[2], 0.03f, 1e-6);
    EXPECT_NEAR(dataCentimeter[4], 0.04f, 1e-6);
    EXPECT_NEAR(dataCentimeter[5], 0.05f, 1e-6);
    EXPECT_NEAR(dataCentimeter[6], 0.06f, 1e-6);

    // Test FOOT
    converter.setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::FOOT);
    pclMsgs.clear();
    converter.toRosMsg(std::make_shared<dai::PointCloudData>(pclData), pclMsgs);
    ASSERT_EQ(pclMsgs.size(), 1);
    const auto& msgFoot = pclMsgs.front();
    const float* dataFoot = reinterpret_cast<const float*>(msgFoot.data.data());
    EXPECT_NEAR(dataFoot[0], 0.3048f, 1e-6);
    EXPECT_NEAR(dataFoot[1], 0.6096f, 1e-6);
    EXPECT_NEAR(dataFoot[2], 0.9144f, 1e-6);
    EXPECT_NEAR(dataFoot[4], 1.2192f, 1e-6);
    EXPECT_NEAR(dataFoot[5], 1.524f, 1e-6);
    EXPECT_NEAR(dataFoot[6], 1.8288f, 1e-6);

    // Test INCH
    converter.setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::INCH);
    pclMsgs.clear();
    converter.toRosMsg(std::make_shared<dai::PointCloudData>(pclData), pclMsgs);
    ASSERT_EQ(pclMsgs.size(), 1);
    const auto& msgInch = pclMsgs.front();
    const float* dataInch = reinterpret_cast<const float*>(msgInch.data.data());
    EXPECT_NEAR(dataInch[0], 0.0254f, 1e-6);
    EXPECT_NEAR(dataInch[1], 0.0508f, 1e-6);
    EXPECT_NEAR(dataInch[2], 0.0762f, 1e-6);
    EXPECT_NEAR(dataInch[4], 0.1016f, 1e-6);
    EXPECT_NEAR(dataInch[5], 0.127f, 1e-6);
    EXPECT_NEAR(dataInch[6], 0.1524f, 1e-6);

    // Test CUSTOM
    converter.setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::CUSTOM);
    pclMsgs.clear();
    converter.toRosMsg(std::make_shared<dai::PointCloudData>(pclData), pclMsgs);
    ASSERT_EQ(pclMsgs.size(), 1);
    const auto& msgCustom = pclMsgs.front();
    const float* dataCustom = reinterpret_cast<const float*>(msgCustom.data.data());
    EXPECT_NEAR(dataCustom[0], 1.0f, 1e-6);
    EXPECT_NEAR(dataCustom[1], 2.0f, 1e-6);
    EXPECT_NEAR(dataCustom[2], 3.0f, 1e-6);
    EXPECT_NEAR(dataCustom[4], 4.0f, 1e-6);
    EXPECT_NEAR(dataCustom[5], 5.0f, 1e-6);
    EXPECT_NEAR(dataCustom[6], 6.0f, 1e-6);
}

TEST(PointCloudConverterTest, ToRosMsgNonColoredPointsWithDepthUnitsTest) {
    dai::PointCloudData pclData;
    pclData.setWidth(2);
    pclData.setHeight(1);
    pclData.setColor(false);
    pclData.setSparse(false);

    std::vector<dai::Point3f> points = {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}};
    pclData.setPoints(points);

    PointCloudConverter converter("test_frame", false);

    // Test MILLIMETER
    converter.setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::MILLIMETER);
    std::deque<sensor_msgs::msg::PointCloud2> pclMsgs;
    converter.toRosMsg(std::make_shared<dai::PointCloudData>(pclData), pclMsgs);
    ASSERT_EQ(pclMsgs.size(), 1);
    const auto& msg = pclMsgs.front();
    const float* data = reinterpret_cast<const float*>(msg.data.data());
    EXPECT_NEAR(data[0], 1.0f, 1e-6);
    EXPECT_NEAR(data[1], 2.0f, 1e-6);
    EXPECT_NEAR(data[2], 3.0f, 1e-6);
    EXPECT_NEAR(data[3], 4.0f, 1e-6);
    EXPECT_NEAR(data[4], 5.0f, 1e-6);
    EXPECT_NEAR(data[5], 6.0f, 1e-6);

    // Test METER
    converter.setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::METER);
    pclMsgs.clear();
    converter.toRosMsg(std::make_shared<dai::PointCloudData>(pclData), pclMsgs);
    ASSERT_EQ(pclMsgs.size(), 1);
    const auto& msgMeter = pclMsgs.front();
    const float* dataMeter = reinterpret_cast<const float*>(msgMeter.data.data());
    EXPECT_NEAR(dataMeter[0], 0.001f, 1e-6);
    EXPECT_NEAR(dataMeter[1], 0.002f, 1e-6);
    EXPECT_NEAR(dataMeter[2], 0.003f, 1e-6);
    EXPECT_NEAR(dataMeter[3], 0.004f, 1e-6);
    EXPECT_NEAR(dataMeter[4], 0.005f, 1e-6);
    EXPECT_NEAR(dataMeter[5], 0.006f, 1e-6);

    // Test CENTIMETER
    converter.setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::CENTIMETER);
    pclMsgs.clear();
    converter.toRosMsg(std::make_shared<dai::PointCloudData>(pclData), pclMsgs);
    ASSERT_EQ(pclMsgs.size(), 1);
    const auto& msgCentimeter = pclMsgs.front();
    const float* dataCentimeter = reinterpret_cast<const float*>(msgCentimeter.data.data());
    EXPECT_NEAR(dataCentimeter[0], 0.01f, 1e-6);
    EXPECT_NEAR(dataCentimeter[1], 0.02f, 1e-6);
    EXPECT_NEAR(dataCentimeter[2], 0.03f, 1e-6);
    EXPECT_NEAR(dataCentimeter[3], 0.04f, 1e-6);
    EXPECT_NEAR(dataCentimeter[4], 0.05f, 1e-6);
    EXPECT_NEAR(dataCentimeter[5], 0.06f, 1e-6);

    // Test FOOT
    converter.setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::FOOT);
    pclMsgs.clear();
    converter.toRosMsg(std::make_shared<dai::PointCloudData>(pclData), pclMsgs);
    ASSERT_EQ(pclMsgs.size(), 1);
    const auto& msgFoot = pclMsgs.front();
    const float* dataFoot = reinterpret_cast<const float*>(msgFoot.data.data());
    EXPECT_NEAR(dataFoot[0], 0.3048f, 1e-6);
    EXPECT_NEAR(dataFoot[1], 0.6096f, 1e-6);
    EXPECT_NEAR(dataFoot[2], 0.9144f, 1e-6);
    EXPECT_NEAR(dataFoot[3], 1.2192f, 1e-6);
    EXPECT_NEAR(dataFoot[4], 1.524f, 1e-6);
    EXPECT_NEAR(dataFoot[5], 1.8288f, 1e-6);

    // Test INCH
    converter.setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::INCH);
    pclMsgs.clear();
    converter.toRosMsg(std::make_shared<dai::PointCloudData>(pclData), pclMsgs);
    ASSERT_EQ(pclMsgs.size(), 1);
    const auto& msgInch = pclMsgs.front();
    const float* dataInch = reinterpret_cast<const float*>(msgInch.data.data());
    EXPECT_NEAR(dataInch[0], 0.0254f, 1e-6);
    EXPECT_NEAR(dataInch[1], 0.0508f, 1e-6);
    EXPECT_NEAR(dataInch[2], 0.0762f, 1e-6);
    EXPECT_NEAR(dataInch[3], 0.1016f, 1e-6);
    EXPECT_NEAR(dataInch[4], 0.127f, 1e-6);
    EXPECT_NEAR(dataInch[5], 0.1524f, 1e-6);

    // Test CUSTOM
    converter.setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::CUSTOM);
    pclMsgs.clear();
    converter.toRosMsg(std::make_shared<dai::PointCloudData>(pclData), pclMsgs);
    ASSERT_EQ(pclMsgs.size(), 1);
    const auto& msgCustom = pclMsgs.front();
    const float* dataCustom = reinterpret_cast<const float*>(msgCustom.data.data());
    EXPECT_NEAR(dataCustom[0], 1.0f, 1e-6);
    EXPECT_NEAR(dataCustom[1], 2.0f, 1e-6);
    EXPECT_NEAR(dataCustom[2], 3.0f, 1e-6);
    EXPECT_NEAR(dataCustom[3], 4.0f, 1e-6);
    EXPECT_NEAR(dataCustom[4], 5.0f, 1e-6);
    EXPECT_NEAR(dataCustom[5], 6.0f, 1e-6);
}
}  // namespace depthai_bridge
