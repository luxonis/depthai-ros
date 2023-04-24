#pragma once

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>

#include "depthai-shared/datatype/RawIMUData.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "ros/time.h"
#include "sensor_msgs/Imu.h"

namespace dai {

namespace ros {

namespace ImuMsgs = sensor_msgs;
using ImuPtr = ImuMsgs::Imu::Ptr;

enum class ImuSyncMethod { COPY, LINEAR_INTERPOLATE_GYRO, LINEAR_INTERPOLATE_ACCEL };

class ImuConverter {
   public:
    ImuConverter(const std::string& frameName,
                 ImuSyncMethod syncMode = ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL,
                 double linear_accel_cov = 0.0,
                 double angular_velocity_cov = 0.0);

    void toRosMsg(std::shared_ptr<dai::IMUData> inData, std::deque<ImuMsgs::Imu>& outImuMsg);

   private:
    void FillImuData_LinearInterpolation(std::vector<IMUPacket>& imuPackets, std::deque<ImuMsgs::Imu>& imuMsgs);
    ImuMsgs::Imu CreateUnitMessage(dai::IMUReportAccelerometer accel, dai::IMUReportGyroscope gyro);
    ImuMsgs::Imu CreateUnitMessage(dai::IMUReportAccelerometer accel, dai::IMUReportGyroscope gyro, dai::IMUReportRotationVectorWAcc rot);

    uint32_t _sequenceNum;
    double _linear_accel_cov, _angular_velocity_cov;
    const std::string _frameName = "";
    ImuSyncMethod _syncMode;
    std::chrono::time_point<std::chrono::steady_clock> _steadyBaseTime;
    ::ros::Time _rosBaseTime;
};

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
