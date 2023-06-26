#pragma once

#include "image_transport/image_transport.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nodelet/nodelet.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "vision_msgs/Detection3DArray.h"
#include "visualization_msgs/MarkerArray.h"

namespace depthai_filters {
class SpatialBB : public nodelet::Nodelet {
   public:
    void onInit() override;

    void overlayCB(const sensor_msgs::ImageConstPtr& preview,
                   const sensor_msgs::CameraInfoConstPtr& info,
                   const vision_msgs::Detection3DArrayConstPtr& detections);

    message_filters::Subscriber<sensor_msgs::Image> previewSub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> infoSub;
    message_filters::Subscriber<vision_msgs::Detection3DArray> detSub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, vision_msgs::Detection3DArray> syncPolicy;
    std::unique_ptr<message_filters::Synchronizer<syncPolicy>> sync;
    ros::Publisher overlayPub;
    ros::Publisher markerPub;
    std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                         "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                         "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};
    bool desqueeze = false;
};
}  // namespace depthai_filters