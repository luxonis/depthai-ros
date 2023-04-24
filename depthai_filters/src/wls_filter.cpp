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
}

void WLSFilter::wlsCB(const sensor_msgs::ImageConstPtr& disp, const sensor_msgs::CameraInfoConstPtr& disp_info, const sensor_msgs::ImageConstPtr& leftImg) {
    cv::Mat leftFrame = utils::msgToMat(leftImg, sensor_msgs::image_encodings::MONO8);
    cv::Mat dispFrame;
    if(disp->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        dispFrame = utils::msgToMat(disp, sensor_msgs::image_encodings::TYPE_16UC1);
    } else {
        dispFrame = utils::msgToMat(disp, sensor_msgs::image_encodings::MONO8);
    }

    cv::Mat dispFiltered;
    sensor_msgs::CameraInfo depthInfo = *disp_info;
    filter->filter(dispFrame, leftFrame, dispFiltered);
    sensor_msgs::Image depth;
    auto factor = (disp_info->K[0] * disp_info->P[3]);
    cv::Mat depthOut = cv::Mat(cv::Size(dispFiltered.cols, dispFiltered.rows), CV_16UC1);
    depthOut.forEach<short>([&dispFiltered, &factor, &disp](short& pixel, const int* position) -> void {
        if(disp->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            auto dispPoint = dispFiltered.at<short>(position);
            if(dispPoint == 0)
                pixel = 0;
            else
                pixel = factor / dispPoint;
        } else {
            auto dispPoint = dispFiltered.at<uint8_t>(position);
            if(dispPoint == 0)
                pixel = 0;
            else
                pixel = factor / dispPoint;
        }
    });

    cv_bridge::CvImage(disp->header, sensor_msgs::image_encodings::TYPE_16UC1, depthOut).toImageMsg(depth);
    depthPub.publish(depth, depthInfo);
}
}  // namespace depthai_filters

#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(depthai_filters::WLSFilter, nodelet::Nodelet)
