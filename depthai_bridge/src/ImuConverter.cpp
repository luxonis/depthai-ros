#include "depthai_bridge/ImuConverter.hpp"

#include "depthai_bridge/depthaiUtility.hpp"

namespace depthai_bridge {

ImuConverter::ImuConverter(const std::string& frameName,
                           ImuSyncMethod syncMode,
                           double linear_accel_cov,
                           double angular_velocity_cov,
                           double rotation_cov,
                           double magnetic_field_cov,
                           bool enable_rotation,
                           bool enable_magn,
                           bool getBaseDeviceTimestamp)
    : BaseConverter(std::move(frameName), getBaseDeviceTimestamp),
      syncMode(syncMode),
      linear_accel_cov(linear_accel_cov),
      angular_velocity_cov(angular_velocity_cov),
      rotation_cov(rotation_cov),
      magnetic_field_cov(magnetic_field_cov),
      enable_rotation(enable_rotation),
      enable_magn(enable_magn),
      sequenceNum(0) {
    if(syncMode != ImuSyncMethod::COPY) {
        DEPTHAI_ROS_WARN_STREAM_ONCE("depthai_bridge", "For RVC4 devices we currently support COPY method");
    }
}

ImuConverter::~ImuConverter() = default;

void ImuConverter::fillImuMsg(ImuMsgs::Imu& msg, dai::IMUReportAccelerometer report) {
    msg.linear_acceleration.x = report.x;
    msg.linear_acceleration.y = report.y;
    msg.linear_acceleration.z = report.z;
    msg.linear_acceleration_covariance = {linear_accel_cov, 0.0, 0.0, 0.0, linear_accel_cov, 0.0, 0.0, 0.0, linear_accel_cov};
}

void ImuConverter::fillImuMsg(ImuMsgs::Imu& msg, dai::IMUReportGyroscope report) {
    msg.angular_velocity.x = report.x;
    msg.angular_velocity.y = report.y;
    msg.angular_velocity.z = report.z;
    msg.angular_velocity_covariance = {angular_velocity_cov, 0.0, 0.0, 0.0, angular_velocity_cov, 0.0, 0.0, 0.0, angular_velocity_cov};
}

void ImuConverter::fillImuMsg(ImuMsgs::Imu& msg, dai::IMUReportRotationVectorWAcc report) {
    if(enable_rotation) {
        msg.orientation.x = report.i;
        msg.orientation.y = report.j;
        msg.orientation.z = report.k;
        msg.orientation.w = report.real;
        msg.orientation_covariance = {rotation_cov, 0.0, 0.0, 0.0, rotation_cov, 0.0, 0.0, 0.0, rotation_cov};
    } else {
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;
        msg.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
}

void ImuConverter::fillImuMsg(ImuMsgs::Imu& msg, dai::IMUReportMagneticField report) {
    return;
}

void ImuConverter::fillImuMsg(depthai_ros_msgs::msg::ImuWithMagneticField& msg, dai::IMUReportAccelerometer report) {
    fillImuMsg(msg.imu, report);
}

void ImuConverter::fillImuMsg(depthai_ros_msgs::msg::ImuWithMagneticField& msg, dai::IMUReportGyroscope report) {
    fillImuMsg(msg.imu, report);
}

void ImuConverter::fillImuMsg(depthai_ros_msgs::msg::ImuWithMagneticField& msg, dai::IMUReportRotationVectorWAcc report) {
    fillImuMsg(msg.imu, report);
}

void ImuConverter::fillImuMsg(depthai_ros_msgs::msg::ImuWithMagneticField& msg, dai::IMUReportMagneticField report) {
    msg.field.magnetic_field.x = report.x;
    msg.field.magnetic_field.y = report.y;
    msg.field.magnetic_field.z = report.z;
    msg.field.magnetic_field_covariance = {magnetic_field_cov, 0.0, 0.0, 0.0, magnetic_field_cov, 0.0, 0.0, 0.0, magnetic_field_cov};
}

void ImuConverter::toRosMsg(std::shared_ptr<dai::IMUData> inData, std::deque<ImuMsgs::Imu>& outImuMsgs) {
    if(updateRosBaseTimeOnToRosMsg) {
        updateRosBaseTime();
    }
    if(syncMode != ImuSyncMethod::COPY) {
        FillImuData_LinearInterpolation(inData->packets, outImuMsgs);
    } else {
        for(int i = 0; i < inData->packets.size(); ++i) {
            auto accel = inData->packets[i].acceleroMeter;
            auto gyro = inData->packets[i].gyroscope;

            ImuMsgs::Imu msg;
            std::chrono::_V2::steady_clock::time_point tstamp;
            if(getBaseDeviceTimestamp)
                tstamp = accel.getTimestampDevice();
            else
                tstamp = accel.getTimestamp();
            if(enable_rotation) {
                auto rot = inData->packets[i].rotationVector;
                CreateUnitMessage(msg, tstamp, accel, gyro, rot);
            } else {
                CreateUnitMessage(msg, tstamp, accel, gyro);
            }

            outImuMsgs.push_back(msg);
        }
    }
}

void ImuConverter::toRosDaiMsg(std::shared_ptr<dai::IMUData> inData, std::deque<depthai_ros_msgs::msg::ImuWithMagneticField>& outImuMsgs) {
    if(updateRosBaseTimeOnToRosMsg) {
        updateRosBaseTime();
    }
    if(syncMode != ImuSyncMethod::COPY) {
        FillImuData_LinearInterpolation(inData->packets, outImuMsgs);
    } else {
        for(int i = 0; i < inData->packets.size(); ++i) {
            auto accel = inData->packets[i].acceleroMeter;
            auto gyro = inData->packets[i].gyroscope;
            auto rot = inData->packets[i].rotationVector;
            auto magn = inData->packets[i].magneticField;
            depthai_ros_msgs::msg::ImuWithMagneticField msg;
            std::chrono::_V2::steady_clock::time_point tstamp;
            if(getBaseDeviceTimestamp)
                tstamp = accel.getTimestampDevice();
            else
                tstamp = accel.getTimestamp();
            CreateUnitMessage(msg, tstamp, accel, gyro, rot, magn);
            outImuMsgs.push_back(msg);
        }
    }
}

}  // namespace depthai_bridge
