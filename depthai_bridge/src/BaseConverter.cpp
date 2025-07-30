#include "depthai_bridge/BaseConverter.hpp"

#include <memory>

#include "depthai/common/CameraExposureOffset.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai_bridge/depthaiUtility.hpp"

namespace depthai_bridge {

BaseConverter::BaseConverter(std::string frameName, bool getBaseDeviceTimestamp)
    : frameName(std::move(frameName)),
      getBaseDeviceTimestamp(getBaseDeviceTimestamp),
      steadyBaseTime(std::chrono::steady_clock::now()),
      rosBaseTime(rclcpp::Clock().now()),
      updateRosBaseTimeOnToRosMsg(false),
      totalNsChange(0) {}

BaseConverter::~BaseConverter() = default;

void BaseConverter::updateRosBaseTime() {
    updateBaseTime(steadyBaseTime, rosBaseTime, totalNsChange);
}

std_msgs::msg::Header BaseConverter::getRosHeader(const std::shared_ptr<dai::Buffer>& inData, bool addExpOffset, dai::CameraExposureOffset offset) {
    if(updateRosBaseTimeOnToRosMsg) {
        updateRosBaseTime();
    }
    std_msgs::msg::Header header;
    header.frame_id = frameName;
    std::chrono::_V2::steady_clock::time_point tstamp;
    if(getBaseDeviceTimestamp)
        if(addExpOffset) {
            auto data = std::dynamic_pointer_cast<dai::ImgFrame>(inData);
            tstamp = data->getTimestampDevice(offset);
        } else
            tstamp = inData->getTimestampDevice();
    else if(addExpOffset) {
        auto data = std::dynamic_pointer_cast<dai::ImgFrame>(inData);
        tstamp = data->getTimestamp(offset);
    } else
        tstamp = inData->getTimestamp();
    header.stamp = getFrameTime(rosBaseTime, steadyBaseTime, tstamp);
    return header;
}
}  // namespace depthai_bridge
