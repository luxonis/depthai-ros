#pragma once

#include <deque>
#include <memory>
#include <string>

#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "sensor_msgs//msg/point_cloud2.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "depthai_bridge/BaseConverter.hpp"

namespace depthai_bridge {

class PointCloudConverter : public BaseConverter {
   public:
    explicit PointCloudConverter(std::string frameName, bool getBaseDeviceTimestamp = false);
    ~PointCloudConverter();

    void toRosMsg(std::shared_ptr<dai::PointCloudData> inPcl, std::deque<sensor_msgs::msg::PointCloud2>& pclMsgs);

};

}  // namespace depthai_bridge
