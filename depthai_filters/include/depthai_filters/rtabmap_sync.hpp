#pragma once

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rtabmap_msgs/msg/rgbd_image.hpp"
#include "depthai_ros_msgs/msg/tracked_features.hpp"

namespace depthai_filters {
class RTABMapSync : public rclcpp::Node {
   public:
    explicit RTABMapSync(const rclcpp::NodeOptions& options);
    void onInit();

    void syncCB(    const sensor_msgs::msg::Image::ConstSharedPtr& rgb,
                   const sensor_msgs::msg::CameraInfo::ConstSharedPtr& rgb_info,
                    const sensor_msgs::msg::Image::ConstSharedPtr& depth,
                   const sensor_msgs::msg::CameraInfo::ConstSharedPtr& depth_info,
                   const depthai_ros_msgs::msg::TrackedFeatures::ConstSharedPtr& features);

    message_filters::Subscriber<sensor_msgs::msg::Image> rgbSub, depthSub;
    message_filters::Subscriber<depthai_ros_msgs::msg::TrackedFeatures> featureSub;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> rgbInfoSub, depthInfoSub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, depthai_ros_msgs::msg::TrackedFeatures>
        syncPolicy;
    std::unique_ptr<message_filters::Synchronizer<syncPolicy>> sync;
    rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbdPub;
    float getDepthAt(int x, int y, const sensor_msgs::msg::Image::ConstSharedPtr& depth_image);

};

}  // namespace depthai_filters