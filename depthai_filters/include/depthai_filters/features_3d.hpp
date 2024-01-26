#pragma once

#include "depthai_ros_msgs/msg/tracked_features.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace depthai_filters {
class Features3D : public rclcpp::Node {
   public:
    explicit Features3D(const rclcpp::NodeOptions& options);
    void onInit();

    void overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& depth,
                   const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info,
                   const depthai_ros_msgs::msg::TrackedFeatures::ConstSharedPtr& features);

    message_filters::Subscriber<sensor_msgs::msg::Image> depthSub;
    message_filters::Subscriber<depthai_ros_msgs::msg::TrackedFeatures> featureSub;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> infoSub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, depthai_ros_msgs::msg::TrackedFeatures>
        syncPolicy;
    std::unique_ptr<message_filters::Synchronizer<syncPolicy>> sync;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlayPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pclPub;
    float getDepthAt(int x, int y, const sensor_msgs::msg::Image::ConstSharedPtr& depth_image);
    bool desqueeze = false;
};

}  // namespace depthai_filters