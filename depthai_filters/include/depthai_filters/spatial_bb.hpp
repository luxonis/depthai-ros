#pragma once

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/latest_time.hpp"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace depthai_filters {
class SpatialBB : public rclcpp::Node {
   public:
    explicit SpatialBB(const rclcpp::NodeOptions& options);
    void onInit();

    void overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& preview,
                   const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info,
                   const vision_msgs::msg::Detection3DArray::ConstSharedPtr& detections);

    message_filters::Subscriber<sensor_msgs::msg::Image> previewSub;
    message_filters::Subscriber<vision_msgs::msg::Detection3DArray> detSub;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> infoSub;

    typedef message_filters::sync_policies::LatestTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, vision_msgs::msg::Detection3DArray> syncPolicy;
    std::unique_ptr<message_filters::Synchronizer<syncPolicy>> sync;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub;
};

}  // namespace depthai_filters
