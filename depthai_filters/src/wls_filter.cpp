#include "depthai_filters/wls_filter.hpp"

#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "depthai_filters/utils.hpp"
#include "depthai_filters/wlsConfig.h"
#include "dynamic_reconfigure/server.h"
#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"

namespace depthai_filters {
void WLSFilter::onInit() {
    auto pNH = getPrivateNodeHandle();
    disparityImgSub.subscribe(pNH, "/stereo/image_raw", 1);
    leftImgSub.subscribe(pNH, "/stereo/left/rect", 1);
    disparityInfoSub.subscribe(pNH, "/stereo/camera_info", 1);
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), disparityImgSub, disparityInfoSub, leftImgSub);
    sync->registerCallback(std::bind(&WLSFilter::wlsCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
    filter->setLambda(8000);
    filter->setSigmaColor(1.5);
    it = std::make_shared<image_transport::ImageTransport>(pNH);
    depthPub = it->advertiseCamera("/stereo/wls/filtered", 1);
    paramServer = std::make_shared<dynamic_reconfigure::Server<wlsConfig>>(pNH);
    paramServer->setCallback(std::bind(&WLSFilter::parameterCB, this, std::placeholders::_1, std::placeholders::_2));
}
void WLSFilter::parameterCB(wlsConfig& config, uint32_t level) {
    filter->setLambda(config.lambda);
    filter->setSigmaColor(config.sigma);
    maxDisparity = config.max_disparity;
}

void WLSFilter::wlsCB(const sensor_msgs::ImageConstPtr& disp, const sensor_msgs::CameraInfoConstPtr& disp_info, const sensor_msgs::ImageConstPtr& leftImg) {
    cv::Mat leftFrame = utils::msgToMat(leftImg, sensor_msgs::image_encodings::MONO8);
    cv::Mat dispFrame;

    dispFrame = utils::msgToMat(disp, disp->encoding);
    cv::Mat dispFiltered;
    sensor_msgs::CameraInfo depthInfo = *disp_info;
    filter->filter(dispFrame, leftFrame, dispFiltered);
    sensor_msgs::Image depth;
    auto factor = abs(depthInfo.P[3]) * 100.0;
    // set distortion to 0
    if(disp->encoding == sensor_msgs::image_encodings::MONO8) {
        auto dispMultiplier = 255.0 / maxDisparity;
        cv::Mat depthOut = cv::Mat(dispFiltered.size(), CV_8UC1);
        depthOut.forEach<uint8_t>([&dispFiltered, &factor, &dispMultiplier](uint8_t& pixel, const int* position) -> void {
            auto disp = dispFiltered.at<uint8_t>(position);
            if(disp == 0) {
                pixel = 0;
            } else {
                pixel = factor / disp * dispMultiplier;
            }
        });
        cv_bridge::CvImage(disp->header, sensor_msgs::image_encodings::MONO8, depthOut).toImageMsg(depth);
        depthPub.publish(depth, depthInfo);
        return;
    } else {
        cv::Mat depthOut = cv::Mat(dispFiltered.size(), CV_16UC1);
        auto dispMultiplier = 255.0 * 255.0 / maxDisparity;
        depthOut.forEach<uint16_t>([&dispFiltered, &factor, &dispMultiplier](uint16_t& pixel, const int* position) -> void {
            auto disp = dispFiltered.at<uint16_t>(position);
            if(disp == 0) {
                pixel = 0;
            } else {
                pixel = factor / disp * dispMultiplier;
            }
        });
        cv_bridge::CvImage(disp->header, sensor_msgs::image_encodings::TYPE_16UC1, depthOut).toImageMsg(depth);
        depthPub.publish(depth, depthInfo);
    }
}
}  // namespace depthai_filters

#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(depthai_filters::WLSFilter, nodelet::Nodelet)
