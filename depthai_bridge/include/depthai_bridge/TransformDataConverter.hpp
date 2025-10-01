#pragma once

#include <depthai/common/Quaterniond.hpp>
#include <deque>
#include <memory>
#include <string>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/device/CalibrationHandler.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai_bridge/BaseConverter.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace depthai_bridge {

class TransformDataConverter : public BaseConverter {
   public:
    explicit TransformDataConverter(std::string frameName, std::string childFrameName, bool getBaseDeviceTimestamp = false);
    ~TransformDataConverter();

    void toRosMsg(std::shared_ptr<dai::TransformData> inOdom, std::deque<nav_msgs::msg::Odometry>& odomMsgs);
    void toRosMsg(std::shared_ptr<dai::TransformData> inOdom, std::deque<geometry_msgs::msg::TransformStamped>& transformMsgs);
    void toRosMsg(std::shared_ptr<dai::TransformData> inOdom, std::deque<geometry_msgs::msg::PoseWithCovarianceStamped>& poseMsgs);
    /*
     * @brief Needed for initial transforms obtained for SLAM, so that Rviz and other nodes do not receive invalid quaternions before correct ones come out.
     */
    void fixQuaternion();
    void setCovariance(std::array<double, 36> covariance);
    void setCovariance(std::vector<double> covariance);

   private:
    std::string childFrameName;
    dai::Quaterniond verifyQuaternion(dai::Quaterniond);
    bool quaternionNeedsFixing;
    std::array<double, 36> cov;
};

}  // namespace depthai_bridge
