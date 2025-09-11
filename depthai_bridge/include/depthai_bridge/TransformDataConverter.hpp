#pragma once

#include <deque>
#include <memory>
#include <string>

#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai_bridge/BaseConverter.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace depthai_bridge {

class TransformDataConverter : public BaseConverter {
   public:
    explicit TransformDataConverter(std::string frameName, std::string childFrameName, bool getBaseDeviceTimestamp = false);
    ~TransformDataConverter();

    void toRosMsg(std::shared_ptr<dai::TransformData> inOdom, std::deque<nav_msgs::msg::Odometry>& odomMsgs);
    void toRosMsg(std::shared_ptr<dai::TransformData> inOdom, std::deque<geometry_msgs::msg::TransformStamped>& transformMsgs);

   private:
    std::string childFrameName;
};

}  // namespace depthai_bridge
