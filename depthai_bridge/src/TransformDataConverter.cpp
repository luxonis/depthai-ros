#include "depthai_bridge/TransformDataConverter.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace depthai_bridge {

TransformDataConverter::TransformDataConverter(std::string frameName, std::string childFrameName, bool getBaseDeviceTimestamp)
    : BaseConverter(std::move(frameName), getBaseDeviceTimestamp), childFrameName(childFrameName), quaternionNeedsFixing(false) {
    cov = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

TransformDataConverter::~TransformDataConverter() = default;
void TransformDataConverter::toRosMsg(std::shared_ptr<dai::TransformData> inTransform, std::deque<nav_msgs::msg::Odometry>& odomMsgs) {
    nav_msgs::msg::Odometry odomMsg;
    odomMsg.header = getRosHeader(inTransform);
    odomMsg.header.frame_id = frameName;
    odomMsg.child_frame_id = childFrameName;
    auto trans = inTransform->getTranslation();
    auto quat = inTransform->getQuaternion();
    if(quaternionNeedsFixing) {
        quat = verifyQuaternion(quat);
    }
    odomMsg.pose.pose.position.x = trans.x;
    odomMsg.pose.pose.position.y = trans.y;
    odomMsg.pose.pose.position.z = trans.z;
    odomMsg.pose.pose.orientation.x = quat.qx;
    odomMsg.pose.pose.orientation.y = quat.qy;
    odomMsg.pose.pose.orientation.z = quat.qz;
    odomMsg.pose.pose.orientation.w = quat.qw;
    odomMsg.pose.covariance = cov;
    odomMsgs.push_back(odomMsg);
}

void TransformDataConverter::toRosMsg(std::shared_ptr<dai::TransformData> inTransform, std::deque<geometry_msgs::msg::TransformStamped>& transformMsgs) {
    geometry_msgs::msg::TransformStamped transformMsg;
    transformMsg.header = getRosHeader(inTransform);
    transformMsg.header.frame_id = frameName;
    transformMsg.child_frame_id = childFrameName;
    auto trans = inTransform->getTranslation();
    auto quat = inTransform->getQuaternion();
    if(quaternionNeedsFixing) {
        quat = verifyQuaternion(quat);
    }
    transformMsg.transform.translation.x = trans.x;
    transformMsg.transform.translation.y = trans.y;
    transformMsg.transform.translation.z = trans.z;
    transformMsg.transform.rotation.x = quat.qx;
    transformMsg.transform.rotation.y = quat.qy;
    transformMsg.transform.rotation.z = quat.qz;
    transformMsg.transform.rotation.w = quat.qw;
    transformMsgs.push_back(transformMsg);
}

void TransformDataConverter::toRosMsg(std::shared_ptr<dai::TransformData> inTransform, std::deque<geometry_msgs::msg::PoseWithCovarianceStamped>& poseMsgs) {
    geometry_msgs::msg::PoseWithCovarianceStamped poseMsg;
    poseMsg.header = getRosHeader(inTransform);
    poseMsg.header.frame_id = frameName;
    auto trans = inTransform->getTranslation();
    auto quat = inTransform->getQuaternion();
    if(quaternionNeedsFixing) {
        quat = verifyQuaternion(quat);
    }
    poseMsg.pose.pose.position.x = trans.x;
    poseMsg.pose.pose.position.y = trans.y;
    poseMsg.pose.pose.position.z = trans.z;
    poseMsg.pose.pose.orientation.x = quat.qx;
    poseMsg.pose.pose.orientation.y = quat.qy;
    poseMsg.pose.pose.orientation.z = quat.qz;
    poseMsg.pose.pose.orientation.w = quat.qw;
    poseMsg.pose.covariance = cov;
    poseMsgs.push_back(poseMsg);
}

void TransformDataConverter::fixQuaternion() {
    quaternionNeedsFixing = true;
}

dai::Quaterniond TransformDataConverter::verifyQuaternion(dai::Quaterniond quat) {
    if(quat.qx == 0.0 && quat.qy == 0.0 && quat.qz == 0.5 && quat.qw == 0.0) {
        quat.qz = 0.0;
        quat.qw = 1.0;
    }
    return quat;
}

void TransformDataConverter::setCovariance(std::array<double, 36> covariance) {
    cov = covariance;
}

void TransformDataConverter::setCovariance(std::vector<double> covariance) {
    std::copy(covariance.begin(), covariance.end(), cov.begin());
}

}  // namespace depthai_bridge
