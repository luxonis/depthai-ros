#pragma once

#include <depthai/pipeline/datatype/TransformData.hpp>
#include <deque>
#include <memory>
#include <string>

#include "depthai_bridge/BaseConverter.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace depthai_bridge {

class OdomConverter : public BaseConverter {
   public:
    explicit OdomConverter(std::string frameName, bool getBaseDeviceTimestamp = false);
    ~OdomConverter();

    void toRosMsg(std::shared_ptr<dai::TransformData> inOdom, std::deque<nav_msgs::msg::Odometry>& odomMsgs);

};

}  // namespace depthai_bridge
