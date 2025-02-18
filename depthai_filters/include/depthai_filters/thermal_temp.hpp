#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace depthai_filters {
class ThermalTemp : public rclcpp::Node {
   public:
    explicit ThermalTemp(const rclcpp::NodeOptions& options);
    void onInit();

    void subCB(const sensor_msgs::msg::Image::ConstSharedPtr& preview);
    void mouseCallback(int event, int x, int y, int flags, void* userdata);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr colorPub;
    int mouseX = 0;
    int mouseY = 0;

};

}  // namespace depthai_filters
