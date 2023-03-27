#include "depthai_filters/wls_filter.hpp"

#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "depthai_filters/utils.hpp"

namespace depthai_filters {

WLSFilter::WLSFilter(const rclcpp::NodeOptions& options) : rclcpp::Node("wls_filter", options) {
    onInit();
}
void WLSFilter::onInit() {
    disparityImgSub.subscribe(this, "stereo/image_raw");
    leftImgSub.subscribe(this, "left/image_raw");
    disparityInfoSub.subscribe(this, "stereo/camera_info");
    sync = std::make_unique<message_filters::Synchronizer<syncPolicy>>(syncPolicy(10), disparityImgSub, disparityInfoSub, leftImgSub);
    sync->registerCallback(std::bind(&WLSFilter::wlsCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
    filter->setLambda(8000);
    filter->setSigmaColor(1.5);
    depthPub = image_transport::create_camera_publisher(this, "wls_filtered");
}

void WLSFilter::wlsCB(const sensor_msgs::msg::Image::ConstSharedPtr& disp,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& disp_info,
                      const sensor_msgs::msg::Image::ConstSharedPtr& leftImg) {
    cv::Mat leftFrame = utils::msgToMat(this->get_logger(), leftImg, sensor_msgs::image_encodings::MONO8);
    cv::Mat dispFrame;
    if(disp->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        dispFrame = utils::msgToMat(this->get_logger(), disp, sensor_msgs::image_encodings::TYPE_16UC1);
    } else {
        dispFrame = utils::msgToMat(this->get_logger(), disp, sensor_msgs::image_encodings::MONO8);
    }

    cv::Mat dispFiltered;
    sensor_msgs::msg::CameraInfo depthInfo = *disp_info;
    filter->filter(dispFrame, leftFrame, dispFiltered);
    sensor_msgs::msg::Image depth;
    auto factor = (disp_info->k[0] * disp_info->p[3]);
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

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::WLSFilter);