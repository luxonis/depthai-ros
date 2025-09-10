
#include <gtest/gtest.h>

#include <deque>
#include <memory>

#include "depthai_bridge/ImuConverter.hpp"

namespace depthai_bridge {

TEST(ImuConverterTest, ToRosMsg) {
    ImuConverter converter("test_frame", ImuSyncMethod::COPY, 0.1, 0.1, 0.1, 0.1, true, true, false);
    auto inData = std::make_shared<dai::IMUData>();
    inData->packets.push_back(dai::IMUPacket());
    dai::IMUReportAccelerometer accReport;
    accReport.accuracy = dai::IMUReportAccelerometer::Accuracy::HIGH;
    accReport.x = 1.0;
    accReport.y = 2.0;
    accReport.z = 3.0;

    inData->packets[0].acceleroMeter = accReport;
    dai::IMUReportGyroscope gyroReport;
    gyroReport.accuracy = dai::IMUReportGyroscope::Accuracy::HIGH;
    gyroReport.x = 4.0;
    gyroReport.y = 5.0;
    gyroReport.z = 6.0;

    inData->packets[0].gyroscope = gyroReport;
    dai::IMUReportRotationVectorWAcc rotReport;
    rotReport.accuracy = dai::IMUReportRotationVectorWAcc::Accuracy::HIGH;
    rotReport.i = 7.0;
    rotReport.j = 8.0;
    rotReport.k = 9.0;
    rotReport.real = 1.0;
    inData->packets[0].rotationVector = rotReport;

    std::deque<ImuMsgs::Imu> outImuMsgs;
    converter.toRosMsg(inData, outImuMsgs);

    ASSERT_EQ(outImuMsgs.size(), 1);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].linear_acceleration.x, 1.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].linear_acceleration.y, 2.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].linear_acceleration.z, 3.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].angular_velocity.x, 4.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].angular_velocity.y, 5.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].angular_velocity.z, 6.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].orientation.x, 7.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].orientation.y, 8.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].orientation.z, 9.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].orientation.w, 1.0);
}

TEST(ImuConverterTest, ToRosDaiMsg) {
    ImuConverter converter("test_frame", ImuSyncMethod::COPY, 0.1, 0.1, 0.1, 0.1, true, true, false);
    auto inData = std::make_shared<dai::IMUData>();
    inData->packets.push_back(dai::IMUPacket());
    dai::IMUReportAccelerometer accReport;
    accReport.accuracy = dai::IMUReportAccelerometer::Accuracy::HIGH;
    accReport.x = 1.0;
    accReport.y = 2.0;
    accReport.z = 3.0;

    inData->packets[0].acceleroMeter = accReport;
    dai::IMUReportGyroscope gyroReport;
    gyroReport.accuracy = dai::IMUReportGyroscope::Accuracy::HIGH;
    gyroReport.x = 4.0;
    gyroReport.y = 5.0;
    gyroReport.z = 6.0;

    inData->packets[0].gyroscope = gyroReport;
    dai::IMUReportRotationVectorWAcc rotReport;
    rotReport.accuracy = dai::IMUReportRotationVectorWAcc::Accuracy::HIGH;
    rotReport.i = 7.0;
    rotReport.j = 8.0;
    rotReport.k = 9.0;
    rotReport.real = 1.0;
    inData->packets[0].rotationVector = rotReport;
    dai::IMUReportMagneticField magReport;
    magReport.accuracy = dai::IMUReportMagneticField::Accuracy::HIGH;
    magReport.x = 11.0;
    magReport.y = 12.0;
    magReport.z = 13.0;
    inData->packets[0].magneticField = magReport;
    std::deque<depthai_ros_msgs::msg::ImuWithMagneticField> outImuMsgs;
    converter.toRosDaiMsg(inData, outImuMsgs);

    ASSERT_EQ(outImuMsgs.size(), 1);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].imu.linear_acceleration.x, 1.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].imu.linear_acceleration.y, 2.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].imu.linear_acceleration.z, 3.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].imu.angular_velocity.x, 4.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].imu.angular_velocity.y, 5.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].imu.angular_velocity.z, 6.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].imu.orientation.x, 7.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].imu.orientation.y, 8.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].imu.orientation.z, 9.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].imu.orientation.w, 1.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].field.magnetic_field.x, 11.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].field.magnetic_field.y, 12.0);
    EXPECT_DOUBLE_EQ(outImuMsgs[0].field.magnetic_field.z, 13.0);
}

}  // namespace depthai_bridge
