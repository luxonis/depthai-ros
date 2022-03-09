#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <unordered_map>

#include "depthai/depthai.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#ifdef IS_ROS2
    #include "rclcpp/rclcpp.hpp"
    #include "sensor_msgs/msg/imu.hpp"
    #include <geometry_msgs/msg/transform_stamped.hpp>
#else
    #include <ros/ros.h>

    #include <boost/make_shared.hpp>

    #include "sensor_msgs/Imu.h"
#endif

namespace dai {

namespace ros {

#ifdef IS_ROS2
namespace ImuMsgs = sensor_msgs::msg;
using ImuPtr = ImuMsgs::Imu::SharedPtr;
namespace rosOrigin = ::rclcpp;
namespace GeometryMsg = geometry_msgs::msg;
#else
namespace ImuMsgs = sensor_msgs;
using ImuPtr = ImuMsgs::Imu::Ptr;
namespace rosOrigin = ::ros;
namespace GeometryMsg = geometry_msgs;
#endif
class ImuConverter {
   public:
#ifdef IS_ROS2
    ImuConverter(const std::string& frameName,
                 std::shared_ptr<rosOrigin::Node> node,
                 const bool publishTransform = false, 
                 const std::string base_frame = "oak-d-base-frame",
                 const std::string world_frame = "map");
#else
    ImuConverter(const std::string& frameName,
                 const bool publishTransform = false, 
                 const std::string base_frame = "oak-d-base-frame",
                 const std::string world_frame = "map");
#endif

    void toRosMsg(std::shared_ptr<dai::IMUData> inData, ImuMsgs::Imu& outImuMsg);
    ImuPtr toRosMsgPtr(const std::shared_ptr<dai::IMUData> inData);

   private:
    void publishtransform(ImuMsgs::Imu& outImuMsg);
    uint32_t _sequenceNum;
    const std::string _frameName = "";
    const bool _publishTransform = false;
    const std::string _base_frame = "";
    const std::string _world_frame = "";
    GeometryMsg::TransformStamped imuToOakBase;
    std::unique_ptr<tf2_ros::Buffer> tfBuffer = nullptr;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tfListener = nullptr;

    #ifdef IS_ROS2
    std::shared_ptr<rosOrigin::Node> _node = nullptr;
    #endif
};

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
