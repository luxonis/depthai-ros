#include "depthai_bridge/OdomConverter.hpp"
#include "nav_msgs/msg/odometry.hpp"


namespace depthai_bridge {

OdomConverter::OdomConverter(std::string frameName, bool getBaseDeviceTimestamp)
    : BaseConverter(std::move(frameName), getBaseDeviceTimestamp) {}

OdomConverter::~OdomConverter() = default;
void OdomConverter::toRosMsg(std::shared_ptr<dai::TransformData> inOdom, std::deque<nav_msgs::msg::Odometry>& odomMsgs) {
    nav_msgs::msg::Odometry odomMsg;
    odomMsg.header = getRosHeader(inOdom);
    odomMsg.header.frame_id = frameName;
    odomMsg.child_frame_id = "odom";
    auto trans = inOdom->getTranslation();
    auto quat = inOdom->getQuaternion();
    odomMsg.pose.pose.position.x = trans.x;
    odomMsg.pose.pose.position.y = trans.y;
    odomMsg.pose.pose.position.z = trans.z;
    odomMsg.pose.pose.orientation.x = quat.qx;
    odomMsg.pose.pose.orientation.y = quat.qy;
    odomMsg.pose.pose.orientation.z = quat.qz;
    odomMsg.pose.pose.orientation.w = quat.qw;
    odomMsgs.push_back(odomMsg);
    // 

}

}  // namespace depthai_bridge
