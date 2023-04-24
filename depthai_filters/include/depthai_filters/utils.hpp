#pragma once

#include "opencv2/core/mat.hpp"
#include "sensor_msgs/Image.h"

namespace depthai_filters {
namespace utils {
cv::Mat msgToMat(const sensor_msgs::ImageConstPtr& img, const std::string& encoding);
}  // namespace utils
}  // namespace depthai_filters