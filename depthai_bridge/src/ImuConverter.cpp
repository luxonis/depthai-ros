
#include <depthai_bridge/ImuConverter.hpp>

namespace dai {

namespace ros {

ImuConverter::ImuConverter(const std::string& frameName) : _frameName(frameName), _sequenceNum(0) {}

template <typename T> T lerp(const T &a, const T &b, const double t) {
  return a * (1.0 - t) + b * t;
}

template <typename T> T lerpImu(const T &a, const T &b, const double t) {
    T res;
    res.x = lerp(a.x, b.x, t);
    res.y = lerp(a.y, b.y, t);
    res.z = lerp(a.z, b.z, t);
    return res;

}
void ImuConverter::FillImuData_LinearInterpolation(std::vector<IMUPacket>& imuPackets, std::deque<ImuMsgs::Imu>& imuMsgs){
    
    int accelSequenceNum = -1, gyroSequenceNum = -1; 
    std::deque<dai::IMUReportAccelerometer> accelHist;
    std::deque<dai::IMUReportGyroscope> gyroHist;
    std::deque<dai::IMUReportRotationVectorWAcc> rotationVecHist;


    for (int i = 0; i < imuPackets.size(); ++i) {
        // IMUPacket currPacket = imuPackets[0];
        if (accelHist.size() == 0){
            accelHist.push_back(imuPackets[i].acceleroMeter);
        }
        else if(accelHist.back().sequence != imuPackets[i].acceleroMeter.sequence){
            accelHist.push_back(imuPackets[i].acceleroMeter);
        }

        if (gyroHist.size() == 0){
            gyroHist.push_back(imuPackets[i].gyroscope);
        }
        else if(gyroHist.back().sequence != imuPackets[i].gyroscope.sequence){
            gyroHist.push_back(imuPackets[i].gyroscope);
        }
    
        if (_syncMode == imuSyncMethod::LINEAR_INTERPOLATE_ACCEL){
            if (accelHist.size < 3){
                continue; 
            }
            else{
                dai::IMUReportAccelerometer accel0, accel1;
                accel0.sequence = -1;
                    ROS_DEBUG_STREAM_NAMED("IMU INTERPOLATION: ", " Interpolating LINEAR_INTERPOLATE_ACCEL mode " << std::endl);
                 while(accelHist.size()){
                    if(accel0.sequence == -1){
                        accel0 = accelHist.front();
                        accelHist.pop_front();
                    }
                    else{
                        accel1 = accelHist.front();
                        accelHist.pop_front();
                        auto dt = accel1.timestamp.get() - accel0.timestamp.get();
                        if(!gyroHist.size()){
                            ROS_WARN_STREAM_NAMED("IMU INTERPOLATION: ", "Gyro data not found. Dropping data");
                        }
                        while(gyroHist.size()){
                            dai::IMUReportGyroscope currGyro = gyroHist.front();
                            gyroHist.pop_front();
                            ROS_DEBUG_STREAM_NAMED("IMU INTERPOLATION: ", "Accel 0: Seq => " << accel0.sequence << std::endl  << "       timeStamp => " << accel0.timestamp.get() << std::endl);
                            ROS_DEBUG_STREAM_NAMED("IMU INTERPOLATION: ", "currGyro 0: Seq => " << currGyro.sequence << std::endl  << "       timeStamp => " << currGyro.timestamp.get() << std::endl);
                            ROS_DEBUG_STREAM_NAMED("IMU INTERPOLATION: ", "Accel 1: Seq => " << accel1.sequence << std::endl  << "       timeStamp => " << accel1.timestamp.get() << std::endl);
                            if (currGyro.timestamp.get() > accel0.timestamp.get()){
                                auto alpha = (currGyro.m_time - accel0.m_time) / dt;
                                dai::IMUReportAccelerometer interpAccel = lerpImu(accel0, accel1, alpha);
                                imuMsgs.push_back(CreateUnitMessage(interpAccel, currGyro))
                            }
                            else{
                                ROS_DEBUG_STREAM_NAMED("IMU INTERPOLATION: ", "Droppinh GYRO due to inconsistent timestamps \n");
                            }

                        }
                        accel0 = accel1;
                    }

                }
            }
        }
        else if(_syncMode == imuSyncMethod::LINEAR_INTERPOLATE_GYRO){
            if (gyroHist.size < 3){
                continue; 
            }
            else{
                dai::IMUReportGyroscope gyro0, gyro1;
                accel0.sequence = -1;
                ROS_DEBUG_STREAM_NAMED("IMU INTERPOLATION: ", " Interpolating LINEAR_INTERPOLATE_GYRO mode " << std::endl);
                // accel1.sequence = -1;
                while(gyroHist.size()){
                    if(gyro0.sequence == -1){
                        gyro0 = gyroHist.front();
                        gyroHist.pop_front();
                    }
                    else{
                        gyro1 = gyroHist.front();
                        gyroHist.pop_front();
                        auto dt = gyro1.timestamp.get() - gyro1.timestamp.get();
                        if(!accelHist.size()){
                            ROS_WARN_STREAM_NAMED("IMU INTERPOLATION: ", "Accel data not found. Dropping data");
                        }
                        while(accelHist.size()){
                            dai::IMUReportAccelerometer currAccel = accelHist.front();
                            accelHist.pop_front();
                            ROS_DEBUG_STREAM_NAMED("IMU INTERPOLATION: ", "gyro 0: Seq => " << gyro0.sequence << std::endl  << "       timeStamp => " << gyro0.timestamp.get() << std::endl);
                            ROS_DEBUG_STREAM_NAMED("IMU INTERPOLATION: ", "currAccel 0: Seq => " << currAccel.sequence << std::endl  << "       timeStamp => " << currAccel.timestamp.get() << std::endl);
                            ROS_DEBUG_STREAM_NAMED("IMU INTERPOLATION: ", "gyro 1: Seq => " << gyro1.sequence << std::endl  << "       timeStamp => " << gyro1.timestamp.get() << std::endl);
                            if (currAccel.timestamp.get() > gyro0.timestamp.get()){
                                auto alpha = (currAccel.m_time - gyro0.m_time) / dt;
                                dai::IMUReportGyroscope interpGyro = lerpImu(gyro0, gyro1, alpha);
                                imuMsgs.push_back(CreateUnitMessage(currAccel, interpGyro))
                            }
                            else{
                                ROS_DEBUG_STREAM_NAMED("IMU INTERPOLATION: ", "Droppinh ACCEL due to inconsistent timestamps \n");
                            }

                        }
                        gyro0 = gyro1;
                    }

                }
            }
        } 




        
        
    }
}

void ImuConverter::toRosMsg(std::shared_ptr<dai::IMUData> inData, std::deque<ImuMsgs::Imu>& outImuMsgs) {


    
// setting the header
#ifndef IS_ROS2
    outImuMsg.header.seq = _sequenceNum;
    outImuMsg.header.stamp = ::ros::Time::now();
#else
    outImuMsg.header.stamp = rclcpp::Clock().now();
#endif

    outImuMsg.header.frame_id = _frameName;

    const auto imuPacket = inData->packets[inData->packets.size() - 1];

    {
        const auto& rVvalues = imuPacket.rotationVector;

        outImuMsg.orientation.x = rVvalues.i;
        outImuMsg.orientation.y = rVvalues.j;
        outImuMsg.orientation.z = rVvalues.k;
        outImuMsg.orientation.w = rVvalues.real;
    }

    {
        const auto& gyroValues = imuPacket.gyroscope;

        outImuMsg.angular_velocity.x = gyroValues.x;
        outImuMsg.angular_velocity.y = gyroValues.y;
        outImuMsg.angular_velocity.z = gyroValues.z;
    }

    {
        const auto& acceleroValues = imuPacket.acceleroMeter;

        outImuMsg.linear_acceleration.x = acceleroValues.x;
        outImuMsg.linear_acceleration.y = acceleroValues.y;
        outImuMsg.linear_acceleration.z = acceleroValues.z;
    }

    _sequenceNum++;
}

ImuPtr ImuConverter::toRosMsgPtr(const std::shared_ptr<dai::IMUData> inData) {
#ifdef IS_ROS2
    ImuPtr ptr = std::make_shared<ImuMsgs::Imu>();
#else
    ImuPtr ptr = boost::make_shared<ImuMsgs::Imu>();
#endif

    toRosMsg(inData, *ptr);
    return ptr;
}

}  // namespace ros
}  // namespace dai