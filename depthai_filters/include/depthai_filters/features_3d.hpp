#pragma once

#include "depthai_ros_msgs/TrackedFeatures.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nodelet/nodelet.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/MarkerArray.h"

namespace depthai_filters {
class Features3D : public nodelet::Nodelet {
   public:
    void onInit();

    void overlayCB(const sensor_msgs::ImageConstPtr& depth,
                   const sensor_msgs::CameraInfoConstPtr& info,
                   const depthai_ros_msgs::TrackedFeaturesConstPtr& features);

    message_filters::Subscriber<sensor_msgs::Image> depthSub;
    message_filters::Subscriber<depthai_ros_msgs::TrackedFeatures> featureSub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> infoSub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, depthai_ros_msgs::TrackedFeatures> syncPolicy;
    std::unique_ptr<message_filters::Synchronizer<syncPolicy>> sync;
    ros::Publisher overlayPub;
    ros::Publisher pclPub;
    float getDepthAt(int x, int y, const sensor_msgs::ImageConstPtr& depth_image);
    bool desqueeze = false;
};

}  // namespace depthai_filters