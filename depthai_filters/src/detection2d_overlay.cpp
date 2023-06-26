#include "depthai_filters/detection2d_overlay.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_filters/utils.hpp"

namespace depthai_filters {

Detection2DOverlay::Detection2DOverlay(const rclcpp::NodeOptions& options) : rclcpp::Node("detection_overlay", options) {
    onInit();
}
void Detection2DOverlay::onInit() {
    previewSub.subscribe(this, "rgb/preview/image_raw");
    detSub.subscribe(this, "nn/detections");
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), previewSub, detSub);
    sync->registerCallback(std::bind(&Detection2DOverlay::overlayCB, this, std::placeholders::_1, std::placeholders::_2));
    overlayPub = this->create_publisher<sensor_msgs::msg::Image>("overlay", 10);
    labelMap = this->declare_parameter<std::vector<std::string>>("label_map", labelMap);
}

void Detection2DOverlay::overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& preview,
                                   const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections) {
    cv::Mat previewMat = utils::msgToMat(this->get_logger(), preview, sensor_msgs::image_encodings::BGR8);

    auto blue = cv::Scalar(255, 0, 0);

    for(auto& detection : detections->detections) {
        auto x1 = detection.bbox.center.position.x - detections->detections[0].bbox.size_x / 2.0;
        auto x2 = detection.bbox.center.position.x + detections->detections[0].bbox.size_x / 2.0;
        auto y1 = detection.bbox.center.position.y - detections->detections[0].bbox.size_y / 2.0;
        auto y2 = detection.bbox.center.position.y + detections->detections[0].bbox.size_y / 2.0;
        auto labelStr = labelMap[stoi(detection.results[0].hypothesis.class_id)];
        auto confidence = detection.results[0].hypothesis.score;
        utils::addTextToFrame(previewMat, labelStr, x1 + 10, y1 + 20);
        std::stringstream confStr;
        confStr << std::fixed << std::setprecision(2) << confidence * 100;
        utils::addTextToFrame(previewMat, confStr.str(), x1 + 10, y1 + 40);
        cv::rectangle(previewMat, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), blue);
    }
    sensor_msgs::msg::Image outMsg;
    cv_bridge::CvImage(preview->header, sensor_msgs::image_encodings::BGR8, previewMat).toImageMsg(outMsg);

    overlayPub->publish(outMsg);
}

}  // namespace depthai_filters
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::Detection2DOverlay);