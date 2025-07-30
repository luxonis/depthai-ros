#pragma once

#include <deque>
#include <memory>
#include <string>

#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai_bridge/BaseConverter.hpp"
#include "sensor_msgs//msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace depthai_bridge {

class PointCloudConverter : public BaseConverter {
   public:
    explicit PointCloudConverter(std::string frameName, bool getBaseDeviceTimestamp = false);
    ~PointCloudConverter();

    /**
     * @brief Set the Depth Unit object. By default mode is set to milimeters.
     *
     * @param depthUnit
     */
    void setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit depthUnit);
    double getScaleFactor() const;
    void toRosMsg(std::shared_ptr<dai::PointCloudData> inPcl, std::deque<sensor_msgs::msg::PointCloud2>& pclMsgs);

   private:
    double scaleFactor;
};

}  // namespace depthai_bridge
