#pragma once

#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace depthai_filters {
class WLSFilter : public rclcpp::Node {
   public:
    explicit WLSFilter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    void onInit();

    void wlsCB(const sensor_msgs::msg::Image::ConstSharedPtr& disp,
               const sensor_msgs::msg::CameraInfo::ConstSharedPtr& disp_info,
               const sensor_msgs::msg::Image::ConstSharedPtr& leftImg);

    message_filters::Subscriber<sensor_msgs::msg::Image> disparityImgSub;
    message_filters::Subscriber<sensor_msgs::msg::Image> leftImgSub;

    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> disparityInfoSub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::Image> syncPolicy;
    std::unique_ptr<message_filters::Synchronizer<syncPolicy>> sync;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> filter;
    image_transport::CameraPublisher depthPub;
};
}  // namespace depthai_filters
