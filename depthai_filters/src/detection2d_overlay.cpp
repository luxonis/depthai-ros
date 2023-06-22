#include "depthai_filters/detection2d_overlay.hpp"

#include <memory>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "depthai_filters/utils.hpp"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"

namespace depthai_filters {
void Detection2DOverlay::onInit() {
    auto pNH = getPrivateNodeHandle();
    previewSub.subscribe(pNH, "/rgb/preview/image_raw", 1);
    detSub.subscribe(pNH, "/nn/detections", 1);
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), previewSub, detSub);
    pNH.getParam("label_map", labelMap);
    sync->registerCallback(std::bind(&Detection2DOverlay::overlayCB, this, std::placeholders::_1, std::placeholders::_2));
    overlayPub = pNH.advertise<sensor_msgs::Image>("overlay", 10);
}

void Detection2DOverlay::overlayCB(const sensor_msgs::ImageConstPtr& preview, const vision_msgs::Detection2DArrayConstPtr& detections) {
    cv::Mat previewMat = utils::msgToMat(preview, sensor_msgs::image_encodings::BGR8);
    auto white = cv::Scalar(255, 255, 255);
    auto black = cv::Scalar(0, 0, 0);
    auto blue = cv::Scalar(255, 0, 0);

    for(auto& detection : detections->detections) {
        auto x1 = detection.bbox.center.x - detections->detections[0].bbox.size_x / 2.0;
        auto x2 = detection.bbox.center.x + detections->detections[0].bbox.size_x / 2.0;
        auto y1 = detection.bbox.center.y - detections->detections[0].bbox.size_y / 2.0;
        auto y2 = detection.bbox.center.y + detections->detections[0].bbox.size_y / 2.0;
        auto labelStr = labelMap[detection.results[0].id];
        auto confidence = detection.results[0].score;
        cv::putText(previewMat, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, white, 3);
        cv::putText(previewMat, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, black);
        std::stringstream confStr;
        confStr << std::fixed << std::setprecision(2) << confidence * 100;
        cv::putText(previewMat, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, white, 3);
        cv::putText(previewMat, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, black);
        cv::rectangle(previewMat, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), blue);
    }
    sensor_msgs::Image outMsg;
    cv_bridge::CvImage(preview->header, sensor_msgs::image_encodings::BGR8, previewMat).toImageMsg(outMsg);

    overlayPub.publish(outMsg);
}
}  // namespace depthai_filters

#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(depthai_filters::Detection2DOverlay, nodelet::Nodelet)
