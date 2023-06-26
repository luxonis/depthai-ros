#pragma once

#include "opencv2/core/mat.hpp"
#include "sensor_msgs/Image.h"

namespace depthai_filters {
namespace utils {
cv::Mat msgToMat(const sensor_msgs::ImageConstPtr& img, const std::string& encoding);
void addTextToFrame(cv::Mat& frame, const std::string& text, int x, int y);
}  // namespace utils
}  // namespace depthai_filters