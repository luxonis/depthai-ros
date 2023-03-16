#include "depthai_filters/utils.hpp"

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"

namespace depthai_filters {
namespace utils {
cv::Mat msgToMat(const rclcpp::Logger& logger, const sensor_msgs::msg::Image::ConstSharedPtr& img, const std::string& encoding) {
    cv::Mat mat;
    try {
        mat = cv_bridge::toCvCopy(img, encoding)->image;
    } catch(cv_bridge::Exception& e) {
        RCLCPP_ERROR(logger, "%s", e.what());
    }
    return mat;
}
}  // namespace utils
}  // namespace depthai_filters