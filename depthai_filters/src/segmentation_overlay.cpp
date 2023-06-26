#include "depthai_filters/segmentation_overlay.hpp"

#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "depthai_filters/utils.hpp"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"

namespace depthai_filters {
void SegmentationOverlay::onInit() {
    auto pNH = getPrivateNodeHandle();
    previewSub.subscribe(pNH, "/rgb/preview/image_raw", 1);
    segSub.subscribe(pNH, "/nn/image_raw", 1);
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), previewSub, segSub);
    pNH.getParam("label_map", labelMap);
    sync->registerCallback(std::bind(&SegmentationOverlay::overlayCB, this, std::placeholders::_1, std::placeholders::_2));
    overlayPub = pNH.advertise<sensor_msgs::Image>("overlay", 10);
}

void SegmentationOverlay::overlayCB(const sensor_msgs::ImageConstPtr& preview, const sensor_msgs::ImageConstPtr& segmentation) {
    cv::Mat previewMat = utils::msgToMat(preview, sensor_msgs::image_encodings::BGR8);
    cv::Mat segMat = utils::msgToMat(segmentation, sensor_msgs::image_encodings::BGR8);

    cv::resize(segMat, segMat, cv::Size(previewMat.cols, previewMat.rows), cv::INTER_LINEAR);
    double alpha = 0.5;
    cv::Mat outImg;
    cv::addWeighted(previewMat, alpha, segMat, alpha, 0.0, outImg);

    sensor_msgs::Image outMsg;
    cv_bridge::CvImage(preview->header, sensor_msgs::image_encodings::BGR8, outImg).toImageMsg(outMsg);

    overlayPub.publish(outMsg);
}
}  // namespace depthai_filters

#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(depthai_filters::SegmentationOverlay, nodelet::Nodelet)
