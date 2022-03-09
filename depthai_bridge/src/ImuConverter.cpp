
#include <depthai_bridge/ImuConverter.hpp>

namespace dai {

namespace ros {
#ifdef IS_ROS2
    ImuConverter::ImuConverter(const std::string& frameName,
                               std::shared_ptr<rosOrigin::Node> node,
                               const bool publishTransform, 
                               const std::string base_frame,
                               const std::string world_frame) 
: _frameName(frameName),
  _node(node),
  _publishTransform(publishTransform), 
  _base_frame(base_frame),
  _world_frame(world_frame), 
  _sequenceNum(0) {
    if(_publishTransform) {
        tfBuffer = std::make_unique<tf2_ros::Buffer>(_node->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
        br = std::make_unique<tf2_ros::TransformBroadcaster>(*node);

        while(!tfBuffer->canTransform(_frameName, _base_frame, tf2::TimePointZero) && rclcpp::ok()) {
            RCLCPP_INFO(node->get_logger(), tfBuffer->allFramesAsString());
            RCLCPP_INFO(
                    node->get_logger(), "waiting for IMU transform to become available");
            rclcpp::sleep_for(std::chrono::milliseconds(30));
        }
        RCLCPP_INFO(node->get_logger(), "transform available");

        imuToOakBase  = tfBuffer->lookupTransform(_frameName, _base_frame, tf2::TimePointZero);
    }
}
#else
    ImuConverter::ImuConverter(const std::string& frameName,
                               const bool publishTransform, 
                               const std::string base_frame,
                               const std::string world_frame) 
    : _frameName(frameName),
      _publishTransform(publishTransform), 
      _base_frame(base_frame),
      _world_frame(world_frame), 
      _sequenceNum(0) {
        if(_publishTransform) {
            tfBuffer = std::make_unique<tf2_ros::Buffer>();
            tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
            br = std::make_unique<tf2_ros::TransformBroadcaster>();
            
            imuToOakBase  = tfBuffer->lookupTransform(_frameName, _base_frame, rosOrigin::Time(0), rosOrigin::Duration(1.0) );
        }
    }
#endif

void ImuConverter::toRosMsg(std::shared_ptr<dai::IMUData> inData, ImuMsgs::Imu& outImuMsg) {
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

    if(_publishTransform) {
        publishtransform(outImuMsg);
    }
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

void ImuConverter::publishtransform(ImuMsgs::Imu& outImuMsg){
    GeometryMsg::TransformStamped transformStamped;

    transformStamped.header.stamp = outImuMsg.header.stamp;
    transformStamped.header.frame_id = _world_frame;

    transformStamped.child_frame_id = _base_frame;

    transformStamped.transform.rotation.x = outImuMsg.orientation.z;
    transformStamped.transform.rotation.y = outImuMsg.orientation.y;
    transformStamped.transform.rotation.z = outImuMsg.orientation.x * -1;
    transformStamped.transform.rotation.w = outImuMsg.orientation.w;

    tf2::doTransform(transformStamped, transformStamped, imuToOakBase);
    transformStamped.header.stamp = outImuMsg.header.stamp;
    transformStamped.header.frame_id = _world_frame;

    transformStamped.child_frame_id = _base_frame;

    br->sendTransform(transformStamped);
}

}  // namespace ros
}  // namespace dai