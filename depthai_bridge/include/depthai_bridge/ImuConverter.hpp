#pragma once

#include <deque>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>

#include "depthai-shared/datatype/RawIMUData.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "depthai_ros_msgs/msg/imu_with_magnetic_field.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

namespace dai {

namespace ros {

namespace ImuMsgs = sensor_msgs::msg;
using ImuPtr = ImuMsgs::Imu::SharedPtr;

enum class ImuSyncMethod { COPY, LINEAR_INTERPOLATE_GYRO, LINEAR_INTERPOLATE_ACCEL };

class ImuConverter {
   public:
    ImuConverter(const std::string& frameName,
                 ImuSyncMethod syncMode = ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL,
                 double linear_accel_cov = 0.0,
                 double angular_velocity_cov = 0.0,
                 double rotation_cov = 0.0,
                 double magnetic_field_cov = 0.0,
                 bool enable_rotation = false,
                 bool getBaseDeviceTimestamp = false);
    ~ImuConverter();

    /**
     * @brief Handles cases in which the ROS time shifts forward or backward
     *  Should be called at regular intervals or on-change of ROS time, depending
     *  on monitoring.
     *
     */
    void updateRosBaseTime();

    /**
     * @brief Commands the converter to automatically update the ROS base time on message conversion based on variable
     *
     * @param update: bool whether to automatically update the ROS base time on message conversion
     */
    void setUpdateRosBaseTimeOnToRosMsg(bool update = true) {
        _updateRosBaseTimeOnToRosMsg = update;
    }

    void toRosMsg(std::shared_ptr<dai::IMUData> inData, std::deque<ImuMsgs::Imu>& outImuMsgs);
    void toRosDaiMsg(std::shared_ptr<dai::IMUData> inData, std::deque<depthai_ros_msgs::msg::ImuWithMagneticField>& outImuMsgs);

    template <typename T>
    T lerp(const T& a, const T& b, const double t) {
        return a * (1.0 - t) + b * t;
    }

    template <typename T>
    T lerpImu(const T& a, const T& b, const double t) {
        T res;
        res.x = lerp(a.x, b.x, t);
        res.y = lerp(a.y, b.y, t);
        res.z = lerp(a.z, b.z, t);
        return res;
    }

   private:
    template <typename T>
    void FillImuData_LinearInterpolation(std::vector<IMUPacket>& imuPackets, std::deque<T>& imuMsgs) {
        static std::deque<dai::IMUReportAccelerometer> accelHist;
        static std::deque<dai::IMUReportGyroscope> gyroHist;
        static std::deque<dai::IMUReportRotationVectorWAcc> rotationHist;
        static std::deque<dai::IMUReportMagneticField> magnHist;

        for(int i = 0; i < imuPackets.size(); ++i) {
            if(accelHist.size() == 0) {
                accelHist.push_back(imuPackets[i].acceleroMeter);
            } else if(accelHist.back().sequence != imuPackets[i].acceleroMeter.sequence) {
                accelHist.push_back(imuPackets[i].acceleroMeter);
            }

            if(gyroHist.size() == 0) {
                gyroHist.push_back(imuPackets[i].gyroscope);
            } else if(gyroHist.back().sequence != imuPackets[i].gyroscope.sequence) {
                gyroHist.push_back(imuPackets[i].gyroscope);
            }

            if(_enable_rotation && rotationHist.size() == 0) {
                rotationHist.push_back(imuPackets[i].rotationVector);
            } else if(_enable_rotation && rotationHist.back().sequence != imuPackets[i].rotationVector.sequence) {
                rotationHist.push_back(imuPackets[i].rotationVector);
            } else {
                rotationHist.resize(accelHist.size());
            }

            if(_enable_rotation && magnHist.size() == 0) {
                magnHist.push_back(imuPackets[i].magneticField);
            } else if(_enable_rotation && magnHist.back().sequence != imuPackets[i].magneticField.sequence) {
                magnHist.push_back(imuPackets[i].magneticField);
            } else {
                magnHist.resize(accelHist.size());
            }

            if(_syncMode == ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL) {
                if(accelHist.size() < 3) {
                    continue;
                } else {
                    interpolate(accelHist, gyroHist, rotationHist, magnHist, imuMsgs);
                }

            } else if(_syncMode == ImuSyncMethod::LINEAR_INTERPOLATE_GYRO) {
                if(gyroHist.size() < 3) {
                    continue;
                } else {
                    interpolate(gyroHist, accelHist, rotationHist, magnHist, imuMsgs);
                }
            }
        }
    }

    uint32_t _sequenceNum;
    double _linear_accel_cov, _angular_velocity_cov, _rotation_cov, _magnetic_field_cov;
    bool _enable_rotation;
    const std::string _frameName = "";
    ImuSyncMethod _syncMode;
    std::chrono::time_point<std::chrono::steady_clock> _steadyBaseTime;
    rclcpp::Time _rosBaseTime;
    bool _getBaseDeviceTimestamp;
    // For handling ROS time shifts and debugging
    int64_t _totalNsChange{0};
    // Whether to update the ROS base time on each message conversion
    bool _updateRosBaseTimeOnToRosMsg{false};

    void fillImuMsg(dai::IMUReportAccelerometer report, ImuMsgs::Imu& msg);
    void fillImuMsg(dai::IMUReportGyroscope report, ImuMsgs::Imu& msg);
    void fillImuMsg(dai::IMUReportRotationVectorWAcc report, ImuMsgs::Imu& msg);
    void fillImuMsg(dai::IMUReportMagneticField report, ImuMsgs::Imu& msg);

    void fillImuMsg(dai::IMUReportAccelerometer report, depthai_ros_msgs::msg::ImuWithMagneticField& msg);
    void fillImuMsg(dai::IMUReportGyroscope report, depthai_ros_msgs::msg::ImuWithMagneticField& msg);
    void fillImuMsg(dai::IMUReportRotationVectorWAcc report, depthai_ros_msgs::msg::ImuWithMagneticField& msg);
    void fillImuMsg(dai::IMUReportMagneticField report, depthai_ros_msgs::msg::ImuWithMagneticField& msg);

    template <typename I, typename S, typename T, typename F, typename M>
    void CreateUnitMessage(I first, S second, T third, F fourth, M& msg, std::chrono::_V2::steady_clock::time_point timestamp) {
        fillImuMsg(first, msg);
        fillImuMsg(second, msg);
        fillImuMsg(third, msg);
        fillImuMsg(fourth, msg);

        msg.header.frame_id = _frameName;

        msg.header.stamp = getFrameTime(_rosBaseTime, _steadyBaseTime, timestamp);
    }

    template <typename I, typename S, typename T, typename F, typename M>
    void interpolate(std::deque<I>& interpolated, std::deque<S>& second, std::deque<T>& third, std::deque<F>& fourth, std::deque<M>& imuMsgs) {
        I interp0, interp1;
        S currSecond;
        T currThird;
        F currFourth;
        interp0.sequence = -1;
        while(interpolated.size()) {
            if(interp0.sequence == -1) {
                interp0 = interpolated.front();
                interpolated.pop_front();
            } else {
                interp1 = interpolated.front();
                interpolated.pop_front();
                // remove std::milli to get in seconds
                std::chrono::duration<double, std::milli> duration_ms = interp1.timestamp.get() - interp0.timestamp.get();
                double dt = duration_ms.count();
                while(second.size()) {
                    currSecond = second.front();
                    currThird = third.front();
                    currFourth = fourth.front();
                    if(currSecond.timestamp.get() > interp0.timestamp.get() && currSecond.timestamp.get() <= interp1.timestamp.get()) {
                        // remove std::milli to get in seconds
                        std::chrono::duration<double, std::milli> diff = currSecond.timestamp.get() - interp0.timestamp.get();
                        const double alpha = diff.count() / dt;
                        I interp = lerpImu(interp0, interp1, alpha);
                        M msg;
                        std::chrono::_V2::steady_clock::time_point tstamp;
                        if(_getBaseDeviceTimestamp)
                            tstamp = currSecond.getTimestampDevice();
                        else
                            tstamp = currSecond.getTimestamp();
                        CreateUnitMessage(interp, currSecond, currThird, currFourth, msg, tstamp);
                        imuMsgs.push_back(msg);
                        second.pop_front();
                        third.pop_front();
                        fourth.pop_front();
                    } else if(currSecond.timestamp.get() > interp1.timestamp.get()) {
                        interp0 = interp1;
                        if(interpolated.size()) {
                            interp1 = interpolated.front();
                            interpolated.pop_front();
                            duration_ms = interp1.timestamp.get() - interp0.timestamp.get();
                            dt = duration_ms.count();
                        } else {
                            break;
                        }
                    } else {
                        second.pop_front();
                        third.pop_front();
                        fourth.pop_front();
                    }
                }
                interp0 = interp1;
            }
        }
        interpolated.push_back(interp0);
    }
};

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
