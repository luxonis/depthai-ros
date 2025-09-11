#include "depthai_bridge/TransformDataConverter.hpp"

#include "nav_msgs/msg/odometry.hpp"

namespace depthai_bridge {

TransformDataConverter::TransformDataConverter(std::string frameName, std::string childFrameName, bool getBaseDeviceTimestamp)
    : BaseConverter(std::move(frameName), getBaseDeviceTimestamp), childFrameName(childFrameName) {}

TransformDataConverter::~TransformDataConverter() = default;
void TransformDataConverter::toRosMsg(std::shared_ptr<dai::TransformData> inOdom, std::deque<nav_msgs::msg::Odometry>& odomMsgs) {
    nav_msgs::msg::Odometry odomMsg;
    odomMsg.header = getRosHeader(inOdom);
    odomMsg.header.frame_id = frameName;
    odomMsg.child_frame_id = childFrameName;
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
