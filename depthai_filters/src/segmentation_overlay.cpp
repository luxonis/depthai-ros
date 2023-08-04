#include "depthai_filters/segmentation_overlay.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_filters/utils.hpp"

namespace depthai_filters {

SegmentationOverlay::SegmentationOverlay(const rclcpp::NodeOptions& options) : rclcpp::Node("segmentation_overlay", options) {
    onInit();
}
void SegmentationOverlay::onInit() {
    previewSub.subscribe(this, "rgb/preview/image_raw");
    segSub.subscribe(this, "nn/image_raw");
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), previewSub, segSub);
    sync->registerCallback(std::bind(&SegmentationOverlay::overlayCB, this, std::placeholders::_1, std::placeholders::_2));
    overlayPub = this->create_publisher<sensor_msgs::msg::Image>("overlay", 10);
    labelMap = this->declare_parameter<std::vector<std::string>>("label_map", labelMap);
}

void SegmentationOverlay::overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& preview, const sensor_msgs::msg::Image::ConstSharedPtr& segmentation) {
    cv::Mat previewMat = utils::msgToMat(this->get_logger(), preview, sensor_msgs::image_encodings::BGR8);
    cv::Mat segMat = utils::msgToMat(this->get_logger(), segmentation, sensor_msgs::image_encodings::BGR8);

    cv::resize(segMat, segMat, cv::Size(previewMat.cols, previewMat.rows), cv::INTER_LINEAR);
    double alpha = 0.5;
    cv::Mat outImg;
    cv::addWeighted(previewMat, alpha, segMat, alpha, 0.0, outImg);

    sensor_msgs::msg::Image outMsg;
    cv_bridge::CvImage(preview->header, sensor_msgs::image_encodings::BGR8, outImg).toImageMsg(outMsg);

    overlayPub->publish(outMsg);
}

}  // namespace depthai_filters
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::SegmentationOverlay);