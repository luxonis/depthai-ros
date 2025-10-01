
#include <gtest/gtest.h>

#include <deque>
#include <memory>

#include "depthai/pipeline/datatype/MapData.hpp"
#include "depthai_bridge/GridMapConverter.hpp"

namespace depthai_bridge {

TEST(GridMapConverterTest, ToRosMsgOccupancyGridTest) {
    // Create a sample MapData object
    auto inMap = std::make_shared<dai::MapData>();
    inMap->map.setWidth(10);
    inMap->map.setHeight(10);
    inMap->minX = 0.0f;
    inMap->minY = 0.0f;
    std::vector<uint8_t> data(100, 0);
    inMap->map.setData(data);

    // Create a GridMapConverter object
    GridMapConverter converter("test_frame", false);

    // Convert the MapData to a ROS message
    std::deque<nav_msgs::msg::OccupancyGrid> mapMsgs;
    converter.toRosMsg(inMap, mapMsgs);

    // Check the size of the deque
    ASSERT_EQ(mapMsgs.size(), 1);

    // Get the first message from the deque
    auto& gridMsg = mapMsgs.front();

    // Check the header
    ASSERT_EQ(gridMsg.header.frame_id, "test_frame");

    // Check the info
    ASSERT_FLOAT_EQ(gridMsg.info.resolution, 0.05f);
    ASSERT_EQ(gridMsg.info.width, 10);
    ASSERT_EQ(gridMsg.info.height, 10);
    ASSERT_FLOAT_EQ(gridMsg.info.origin.position.x, 0.0f);
    ASSERT_FLOAT_EQ(gridMsg.info.origin.position.y, 0.0f);
    ASSERT_FLOAT_EQ(gridMsg.info.origin.position.z, 0.0f);
    ASSERT_FLOAT_EQ(gridMsg.info.origin.orientation.x, 0.0f);
    ASSERT_FLOAT_EQ(gridMsg.info.origin.orientation.y, 0.0f);
    ASSERT_FLOAT_EQ(gridMsg.info.origin.orientation.z, 0.0f);
    ASSERT_FLOAT_EQ(gridMsg.info.origin.orientation.w, 1.0f);

    // Check the data
    ASSERT_EQ(gridMsg.data.size(), 100);
    for(int i = 0; i < 100; ++i) {
        ASSERT_EQ(gridMsg.data[i], 0);
    }
}

}  // namespace depthai_bridge
