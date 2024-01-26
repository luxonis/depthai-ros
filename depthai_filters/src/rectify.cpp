#include "depthai_filters/rectify.hpp"

#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "depthai_filters/utils.hpp"

namespace depthai_filters {

Rectify::Rectify(const rclcpp::NodeOptions& options) : rclcpp::Node("rectify", options) {
    onInit();
}
void Rectify::onInit() {
    imgSub.subscribe(this, "image");
    infoSub.subscribe(this, "camera_info");
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), imgSub, infoSub);
    sync->registerCallback(std::bind(&Rectify::rectCB, this, std::placeholders::_1, std::placeholders::_2));
    rectPub = image_transport::create_publisher(this, "image_rect");
}

void Rectify::rectCB(const sensor_msgs::msg::Image::ConstSharedPtr& img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info) {
    
  // Update the camera model
  model_.fromCameraInfo(info);

  // Create cv::Mat views onto both buffers
  const cv::Mat image = cv_bridge::toCvShare(img)->image;
  cv::Mat rect;

  // Rectify and publish
  model_.rectifyImage(image, rect, 1);

  // Allocate new rectified image message
  sensor_msgs::msg::Image::SharedPtr rect_msg =
    cv_bridge::CvImage(img->header, img->encoding, rect).toImageMsg();
  rectPub.publish(rect_msg);

}
}  // namespace depthai_filters

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::Rectify);