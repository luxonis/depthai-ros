#include "depthai_filters/detection2d_overlay.hpp"

#include <memory>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "depthai_filters/utils.hpp"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"
#include "sensor_msgs/CameraInfo.h"

namespace depthai_filters {
void Detection2DOverlay::onInit() {
    auto pNH = getPrivateNodeHandle();
    previewSub.subscribe(pNH, "/rgb/preview/image_raw", 1);
    detSub.subscribe(pNH, "/nn/detections", 1);
    infoSub.subscribe(pNH, "rgb/preview/camera_info", 1);
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), previewSub, infoSub, detSub);
    pNH.getParam("label_map", labelMap);
    pNH.getParam("desqueeze", desqueeze);
    sync->registerCallback(std::bind(&Detection2DOverlay::overlayCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    overlayPub = pNH.advertise<sensor_msgs::Image>("overlay", 10);
}

void Detection2DOverlay::overlayCB(const sensor_msgs::ImageConstPtr& preview,
                                   const sensor_msgs::CameraInfoConstPtr& info,
                                   const vision_msgs::Detection2DArrayConstPtr& detections) {
    cv::Mat previewMat = utils::msgToMat(preview, sensor_msgs::image_encodings::BGR8);
    auto white = cv::Scalar(255, 255, 255);
    auto black = cv::Scalar(0, 0, 0);
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
        auto x1 = detection.bbox.center.x - detections->detections[0].bbox.size_x / 2.0;
        auto x2 = detection.bbox.center.x + detections->detections[0].bbox.size_x / 2.0;
        auto y1 = detection.bbox.center.y - detections->detections[0].bbox.size_y / 2.0;
        auto y2 = detection.bbox.center.y + detections->detections[0].bbox.size_y / 2.0;
        auto labelStr = labelMap[detection.results[0].id];
        auto confidence = detection.results[0].score;
        x1 = x1 * ratioX + offsetX;
        x2 = x2 * ratioX + offsetX;
        y1 = y1 * ratioY + offsetY;
        y2 = y2 * ratioY + offsetY;
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
