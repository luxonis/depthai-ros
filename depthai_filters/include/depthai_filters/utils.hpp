#pragma once

#include "sensor_msgs/msg/image.hpp"
#include "opencv2/core/mat.hpp"

namespace rclcpp {
class Logger;
}

namespace depthai_filters {
namespace utils {
cv::Mat msgToMat(const rclcpp::Logger& logger, const sensor_msgs::msg::Image::ConstSharedPtr& img, const std::string& encoding);
}  // namespace utils
}  // namespace depthai_filters