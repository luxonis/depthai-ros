#include "depthai_filters/detection2d_overlay.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include "depthai_filters/utils.hpp"

namespace depthai_filters {

Detection2DOverlay::Detection2DOverlay(const rclcpp::NodeOptions& options) : rclcpp::Node("detection_overlay", options) {
    onInit();
}
void Detection2DOverlay::onInit() {
    previewSub.subscribe(this, "rgb/preview/image_raw");
	infoSub.subscribe(this, "rgb/preview/camera_info");
    detSub.subscribe(this, "nn/detections");
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), previewSub, infoSub, detSub);
    sync->registerCallback(std::bind(&Detection2DOverlay::overlayCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    overlayPub = this->create_publisher<sensor_msgs::msg::Image>("overlay", 10);
    desqueeze = this->declare_parameter<bool>("desqueeze", false);
    labelMap = this->declare_parameter<std::vector<std::string>>("label_map", labelMap);
}

void Detection2DOverlay::overlayCB(const sensor_msgs::msg::Image::ConstSharedPtr& preview,
								   const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info,
                                   const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections) {
	    cv::Mat previewMat = utils::msgToMat(this->get_logger(), preview, sensor_msgs::image_encodings::BGR8);

    auto blue = cv::Scalar(255, 0, 0);

    double ratioX = 1.0;
    double ratioY = 1.0;
    int offsetX = 0;
    double offsetY = 0;
    // if preview size is less than camera info size
    if(previewMat.rows < info->height || previewMat.cols < info->width) {
        ratioY = double(info->height) / double(previewMat.rows);
        if(desqueeze) {
            ratioX = double(info->width) / double(previewMat.cols);
        } else {
            ratioX = ratioY;
            offsetX = (info->width - info->height) / 2.0;
        }
    } else {
        ratioY = double(previewMat.rows) / double(info->height);
        if(desqueeze) {
            ratioX = double(previewMat.cols) / double(info->width);
        } else {
            ratioX = double(previewMat.cols) / double(info->width);
        }
    }
    for(auto& detection : detections->detections) {
        auto x1 = detection.bbox.center.position.x - detections->detections[0].bbox.size_x / 2.0;
        auto x2 = detection.bbox.center.position.x + detections->detections[0].bbox.size_x / 2.0;
        auto y1 = detection.bbox.center.position.y - detections->detections[0].bbox.size_y / 2.0;
        auto y2 = detection.bbox.center.position.y + detections->detections[0].bbox.size_y / 2.0;
        auto labelStr = labelMap[stoi(detection.results[0].hypothesis.class_id)];
        auto confidence = detection.results[0].hypothesis.score;
        x1 = x1 * ratioX + offsetX;
        x2 = x2 * ratioX + offsetX;
        y1 = y1 * ratioY + offsetY;
        y2 = y2 * ratioY + offsetY;
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
