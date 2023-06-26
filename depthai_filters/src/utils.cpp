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
void addTextToFrame(cv::Mat& frame, const std::string& text, int x, int y) {
    auto white = cv::Scalar(255, 255, 255);
    auto black = cv::Scalar(0, 0, 0);

    cv::putText(frame, text, cv::Point(x, y), cv::FONT_HERSHEY_TRIPLEX, 0.5, white, 3);
    cv::putText(frame, text, cv::Point(x, y), cv::FONT_HERSHEY_TRIPLEX, 0.5, black);
}
}  // namespace utils
}  // namespace depthai_filters