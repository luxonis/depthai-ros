#include "depthai_filters/utils.hpp"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/mat.hpp"
#include "ros/console.h"
#include "sensor_msgs/Image.h"

namespace depthai_filters {
namespace utils {
cv::Mat msgToMat(const sensor_msgs::ImageConstPtr& img, const std::string& encoding) {
    cv::Mat mat;
    try {
        mat = cv_bridge::toCvCopy(img, encoding)->image;
    } catch(cv_bridge::Exception& e) {
        ROS_ERROR("%s", e.what());
    }
    return mat;
}
}  // namespace utils
}  // namespace depthai_filters