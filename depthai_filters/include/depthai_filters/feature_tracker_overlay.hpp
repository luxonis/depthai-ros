#pragma once

#include "cv_bridge/cv_bridge.h"
#include "depthai_ros_msgs/msg/tracked_features.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace depthai_filters {

class FeatureTrackerOverlay : public rclcpp::Node {
   public:
    explicit FeatureTrackerOverlay(const rclcpp::NodeOptions& options);
    void onInit();

    void overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& img, const depthai_ros_msgs::msg::TrackedFeatures::ConstSharedPtr& detections);

    message_filters::Subscriber<sensor_msgs::msg::Image> imgSub;
    message_filters::Subscriber<depthai_ros_msgs::msg::TrackedFeatures> featureSub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, depthai_ros_msgs::msg::TrackedFeatures> syncPolicy;
    std::unique_ptr<message_filters::Synchronizer<syncPolicy>> sync;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlayPub;

    using featureIdType = decltype(geometry_msgs::msg::Point::x);

   private:
    void trackFeaturePath(std::vector<depthai_ros_msgs::msg::TrackedFeature>& features);

    void drawFeatures(cv::Mat& img);

    int circleRadius = 2;
    int maxTrackedFeaturesPathLength = 30;

    cv::Scalar lineColor = cv::Scalar(200, 0, 200);
    cv::Scalar pointColor = cv::Scalar(0, 0, 255);

    int trackedFeaturesPathLength = 10;
    std::unordered_set<featureIdType> trackedIDs;
    std::unordered_map<featureIdType, std::deque<geometry_msgs::msg::Point>> trackedFeaturesPath;
};

}  // namespace depthai_filters