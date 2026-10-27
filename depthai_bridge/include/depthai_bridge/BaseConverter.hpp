#pragma once

#include <chrono>
#include <string>

#include "depthai/common/CameraExposureOffset.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"

namespace depthai_bridge {

class BaseConverter {
   public:
    explicit BaseConverter(std::string frameName, bool getBaseDeviceTimestamp = false);
    virtual ~BaseConverter();

    /**
     * @brief Handles cases in which the ROS time shifts forward or backward
     *  Should be called at regular intervals or on-change of ROS time, depending
     *  on monitoring.
     *
     */
    void updateRosBaseTime();

    /**
     * @brief Commands the converter to automatically update the ROS base time on message conversion based on variable
     *
     * @param update: bool whether to automatically update the ROS base time on message conversion
     */
    void setUpdateRosBaseTimeOnToRosMsg(bool update = true) {
        updateRosBaseTimeOnToRosMsg = update;
    }
    std_msgs::msg::Header getRosHeader(const std::shared_ptr<dai::Buffer>& inData,
                                       bool addExpOffset = false,
                                       dai::CameraExposureOffset offset = dai::CameraExposureOffset());

    std::string getFrameName() const {
        return frameName;
    }
    bool isGetBaseDeviceTimestamp() const {
        return getBaseDeviceTimestamp;
    }
    bool isUpdateRosBaseTimeOnToRosMsg() const {
        return updateRosBaseTimeOnToRosMsg;
    }

   protected:
    const std::string frameName;
    std::chrono::time_point<std::chrono::steady_clock> steadyBaseTime;
    rclcpp::Time rosBaseTime;
    bool getBaseDeviceTimestamp;
    int64_t totalNsChange;
    bool updateRosBaseTimeOnToRosMsg;
};

}  // namespace depthai_bridge
