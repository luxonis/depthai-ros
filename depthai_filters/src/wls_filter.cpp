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
    filter->setLambda(this->declare_parameter<double>("lambda", 8000.0));
    filter->setSigmaColor(this->declare_parameter<double>("sigma_color", 1.5));
    maxDisparity = this->declare_parameter<double>("max_disparity", 760.0);
    paramCBHandle = this->add_on_set_parameters_callback(std::bind(&WLSFilter::parameterCB, this, std::placeholders::_1));
    depthPub = image_transport::create_camera_publisher(this, "wls_filtered");
}

rcl_interfaces::msg::SetParametersResult WLSFilter::parameterCB(const std::vector<rclcpp::Parameter>& params) {
    for(const auto& p : params) {
        if(p.get_name() == "lambda") {
            filter->setLambda(p.as_double());
        } else if(p.get_name() == "sigma_color") {
            filter->setSigmaColor(p.as_double());
        } else if(p.get_name() == "max_disparity") {
            maxDisparity = p.as_double();
        }
    }
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    return res;
}

void WLSFilter::wlsCB(const sensor_msgs::msg::Image::ConstSharedPtr& disp,
                      const sensor_msgs::msg::CameraInfo::ConstSharedPtr& disp_info,
                      const sensor_msgs::msg::Image::ConstSharedPtr& leftImg) {
    cv::Mat leftFrame = utils::msgToMat(this->get_logger(), leftImg, sensor_msgs::image_encodings::MONO8);
    cv::Mat dispFrame;

    dispFrame = utils::msgToMat(this->get_logger(), disp, disp->encoding);
    cv::Mat dispFiltered;
    sensor_msgs::msg::CameraInfo depthInfo = *disp_info;
    filter->filter(dispFrame, leftFrame, dispFiltered);
    sensor_msgs::msg::Image depth;
    auto factor = abs(depthInfo.p[3]) * 100.0;
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

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::WLSFilter);