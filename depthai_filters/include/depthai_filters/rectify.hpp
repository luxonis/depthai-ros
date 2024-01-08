#pragma once

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_geometry/pinhole_camera_model.h"
#include "image_transport/publisher.hpp"
#include "image_transport/image_transport.hpp"

namespace depthai_filters {
class Rectify : public rclcpp::Node {
   public:
    explicit Rectify(const rclcpp::NodeOptions& options);
    void onInit();

    void rectCB(const sensor_msgs::msg::Image::ConstSharedPtr& img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info);

    message_filters::Subscriber<sensor_msgs::msg::Image> imgSub;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> infoSub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> syncPolicy;
    std::unique_ptr<message_filters::Synchronizer<syncPolicy>> sync;
    image_transport::Publisher rectPub;
    image_geometry::PinholeCameraModel model_;
};

}  // namespace depthai_filters