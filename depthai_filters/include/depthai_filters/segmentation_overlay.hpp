#pragma once

#include "image_transport/image_transport.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nodelet/nodelet.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

namespace depthai_filters {
class SegmentationOverlay : public nodelet::Nodelet {
   public:
    void onInit() override;

    void overlayCB(const sensor_msgs::ImageConstPtr& preview, const sensor_msgs::ImageConstPtr& detections);

    message_filters::Subscriber<sensor_msgs::Image> previewSub;
    message_filters::Subscriber<sensor_msgs::Image> segSub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    std::unique_ptr<message_filters::Synchronizer<syncPolicy>> sync;
    ros::Publisher overlayPub;
    std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                         "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                         "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};
};
}  // namespace depthai_filters