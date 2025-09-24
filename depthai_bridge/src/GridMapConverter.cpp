#include "depthai_bridge/GridMapConverter.hpp"

#include "depthai_bridge/depthaiUtility.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace depthai_bridge {

GridMapConverter::GridMapConverter(std::string frameName, bool getBaseDeviceTimestamp) : BaseConverter(std::move(frameName), getBaseDeviceTimestamp) {}

GridMapConverter::~GridMapConverter() = default;

void GridMapConverter::toRosMsg(std::shared_ptr<dai::MapData> inMap, std::deque<nav_msgs::msg::OccupancyGrid>& mapMsgs) {
    nav_msgs::msg::OccupancyGrid grid;
    float cellSize = 0.05f;
    grid.header = getRosHeader(inMap);
    grid.info.resolution = cellSize;
    int width = inMap->map.getWidth();
    int height = inMap->map.getHeight();
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position.x = inMap->minX;
    grid.info.origin.position.y = inMap->minY;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(inMap->map.getData().size());
    // Flip the image along the X axis
    auto data = inMap->map.getData();
    for(int y = 0; y < height; ++y) {
        for(int x = 0; x < width; ++x) {
            grid.data[y * width + x] = data[(height - 1 - y) * width + x];
        }
    }
    mapMsgs.push_back(grid);
}

}  // namespace depthai_bridge
