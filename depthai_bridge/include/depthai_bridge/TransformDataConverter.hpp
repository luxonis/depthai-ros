#pragma once

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
    void convertFromImuFrameToSocket(dai::CameraBoardSocket socket, dai::CalibrationHandler calHandler);

   private:
    std::vector<std::vector<float>> convertMatrixToFloat(const std::array<std::array<double,4>,4>& matrix);
    std::vector<std::vector<float>> multiplyMatrices(const std::vector<std::vector<float>>& a, const std::vector<std::vector<float>>& b);
    std::string childFrameName;
    dai::CameraBoardSocket socketToTransformTo;
    dai::CalibrationHandler calHandler;
    std::shared_ptr<dai::TransformData> transformToSocket;
    bool convertToSocket;
    std::shared_ptr<dai::TransformData> convertToFrame(std::shared_ptr<dai::TransformData> transform);
};

}  // namespace depthai_bridge
