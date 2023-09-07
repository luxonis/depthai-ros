#pragma once

#include <unordered_map>
#include <unordered_set>

#include "cv_bridge/cv_bridge.h"
#include "depthai_ros_msgs/TrackedFeatures.h"
#include "geometry_msgs/Point.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nodelet/nodelet.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

namespace depthai_filters {

class FeatureTrackerOverlay : public nodelet::Nodelet {
   public:
    void onInit() override;

    void overlayCB(const sensor_msgs::ImageConstPtr& img, const depthai_ros_msgs::TrackedFeaturesConstPtr& detections);

    message_filters::Subscriber<sensor_msgs::Image> imgSub;
    message_filters::Subscriber<depthai_ros_msgs::TrackedFeatures> featureSub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, depthai_ros_msgs::TrackedFeatures> syncPolicy;
    std::unique_ptr<message_filters::Synchronizer<syncPolicy>> sync;
    ros::Publisher overlayPub;

    using featureIdType = decltype(geometry_msgs::Point::x);

   private:
    void trackFeaturePath(std::vector<depthai_ros_msgs::TrackedFeature>& features);

    void drawFeatures(cv::Mat& img);

    int circleRadius = 2;
    int maxTrackedFeaturesPathLength = 30;

    cv::Scalar lineColor = cv::Scalar(200, 0, 200);
    cv::Scalar pointColor = cv::Scalar(0, 0, 255);

    int trackedFeaturesPathLength = 10;
    std::unordered_set<featureIdType> trackedIDs;
    std::unordered_map<featureIdType, std::deque<geometry_msgs::Point>> trackedFeaturesPath;
};

}  // namespace depthai_filters