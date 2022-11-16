#include <sensor_msgs/Image.h>

#include <depthai_bridge/ImageConverter.hpp>
#include <opencv2/opencv.hpp>

#include "ros/ros.h"

// Inludes common necessary includes for development using depthai library

dai::rosBridge::ImageConverter inputConverter(true);

void rgbCallback(const sensor_msgs::ImagePtr& rgbImageMsg) {
    cv::Mat rgbImage = inputConverter.rosMsgtoCvMat(*rgbImageMsg);
    cv::imshow("video", rgbImage);
    cv::waitKey(1);
    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rgb_subscriber_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("rgb_image", 5, rgbCallback);
    ros::spin();

    return 0;
}
