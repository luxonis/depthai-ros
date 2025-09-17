#pragma once

#include <deque>
#include <memory>
#include <string>

#include "depthai/pipeline/datatype/MapData.hpp"
#include "depthai_bridge/BaseConverter.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace depthai_bridge {

class GridMapConverter : public BaseConverter {
   public:
    explicit GridMapConverter(std::string frameName, bool getBaseDeviceTimestamp = false);
    ~GridMapConverter();

    void toRosMsg(std::shared_ptr<dai::MapData> inMap, std::deque<nav_msgs::msg::OccupancyGrid>& mapMsgs);
};

}  // namespace depthai_bridge
