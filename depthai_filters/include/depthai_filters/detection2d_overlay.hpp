#pragma once

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace depthai_filters {
class Detection2DOverlay : public rclcpp::Node {
   public:
    explicit Detection2DOverlay(const rclcpp::NodeOptions& options);
    void onInit();

    void overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& preview, const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections);

    message_filters::Subscriber<sensor_msgs::msg::Image> previewSub;
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> detSub;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, vision_msgs::msg::Detection2DArray> syncPolicy;
    std::unique_ptr<message_filters::Synchronizer<syncPolicy>> sync;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr overlayPub;
    std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                         "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                         "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};
};

}  // namespace depthai_filters