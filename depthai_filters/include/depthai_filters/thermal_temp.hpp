#pragma once

#include "image_transport/image_transport.h"
#include "nodelet/nodelet.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

namespace depthai_filters {
class ThermalTemp : public nodelet::Nodelet {
   public:
    void onInit() override;

    void subCB(const sensor_msgs::ImageConstPtr& img);
    void mouseCallback(int event, int x, int y, int flags, void* userdata);
    image_transport::Subscriber sub;
    ros::Publisher colorPub;
    int mouseX = 0;
    int mouseY = 0;
};
}  // namespace depthai_filters
