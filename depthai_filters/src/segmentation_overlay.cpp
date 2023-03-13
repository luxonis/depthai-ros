#include "depthai_filters/segmentation_overlay.hpp"

#include "cv_bridge/cv_bridge.h"

namespace depthai_filters {

SegmentationOverlay::SegmentationOverlay(const rclcpp::NodeOptions& options) : rclcpp::Node("detection_overlay", options) {
    onInit();
}
void SegmentationOverlay::onInit() {
    previewSub.subscribe(this, "/oak/rgb/preview/image_raw");
    segSub.subscribe(this, "/oak/nn/image_raw");
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), previewSub, segSub);
    sync->registerCallback(std::bind(&SegmentationOverlay::overlayCB, this, std::placeholders::_1, std::placeholders::_2));
    overlayPub = this->create_publisher<sensor_msgs::msg::Image>("overlay", 10);
}

void SegmentationOverlay::overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& preview, const sensor_msgs::msg::Image::ConstSharedPtr& segmentation) {
    cv::Mat previewMat, segMat, outImg;
    static const std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                                      "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                                      "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};

    cv_bridge::CvImagePtr previewPtr, segPtr;
    try {
        previewPtr = cv_bridge::toCvCopy(preview, sensor_msgs::image_encodings::BGR8);
        previewMat = previewPtr->image;
        segPtr = cv_bridge::toCvCopy(segmentation, sensor_msgs::image_encodings::BGR8);
        segMat = segPtr->image;
    } catch(cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    }
    resize(segMat, segMat, cv::Size(previewMat.cols, previewMat.rows), cv::INTER_LINEAR);
    double alpha = 0.5;
    addWeighted( previewMat, alpha, segMat, alpha, 0.0, outImg);

    sensor_msgs::msg::Image outMsg;
    cv_bridge::CvImage(preview->header, sensor_msgs::image_encodings::BGR8, outImg).toImageMsg(outMsg);

    overlayPub->publish(outMsg);
}

}  // namespace depthai_filters
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::SegmentationOverlay);