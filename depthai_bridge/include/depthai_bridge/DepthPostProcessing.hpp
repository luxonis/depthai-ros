#pragma once

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/DisparityConverter.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <iostream>
#include <memory>
#include <string>

#include "depthai/depthai.hpp"
#include "depthai_bridge/RosParameters.hpp"

#ifndef IS_ROS2

    #include "depthai_ros_msgs/SetDecimation.h"
    #include "depthai_ros_msgs/SetMedian.h"
    #include "depthai_ros_msgs/SetPostProcessing.h"
    #include "depthai_ros_msgs/SetSpatial.h"
    #include "depthai_ros_msgs/SetSpeckle.h"
    #include "depthai_ros_msgs/SetTemporal.h"
    #include "depthai_ros_msgs/SetThreshold.h"
    #include "ros/ros.h"

using pp_req_msg = depthai_ros_msgs::SetPostProcessing::Request&;
using pp_rep_msg = depthai_ros_msgs::SetPostProcessing::Response&;
using med_req_msg = depthai_ros_msgs::SetMedian::Request&;
using med_rep_msg = depthai_ros_msgs::SetMedian::Response&;
using spk_req_msg = depthai_ros_msgs::SetSpeckle::Request&;
using spk_rep_msg = depthai_ros_msgs::SetSpeckle::Response&;
using tmp_req_msg = depthai_ros_msgs::SetTemporal::Request&;
using tmp_rep_msg = depthai_ros_msgs::SetTemporal::Response&;
using spt_req_msg = depthai_ros_msgs::SetSpatial::Request&;
using spt_rep_msg = depthai_ros_msgs::SetSpatial::Response&;
using trh_req_msg = depthai_ros_msgs::SetThreshold::Request&;
using trh_rep_msg = depthai_ros_msgs::SetThreshold::Response&;
using dcm_req_msg = depthai_ros_msgs::SetDecimation::Request&;
using dcm_rep_msg = depthai_ros_msgs::SetDecimation::Response&;
using ros_node = ros::NodeHandle&;
    #define req_get(x) (request.x)
    #define rep_get(x) (response.x)
    #define set_parameter(a, b) getParamWithWarning(node, a, b)

#else

    #include "depthai_ros_msgs/srv/set_decimation.hpp"
    #include "depthai_ros_msgs/srv/set_median.hpp"
    #include "depthai_ros_msgs/srv/set_post_processing.hpp"
    #include "depthai_ros_msgs/srv/set_spatial.hpp"
    #include "depthai_ros_msgs/srv/set_speckle.hpp"
    #include "depthai_ros_msgs/srv/set_temporal.hpp"
    #include "depthai_ros_msgs/srv/set_threshold.hpp"
    #define req_type void
using pp_req_msg = const std::shared_ptr<depthai_ros_msgs::srv::SetPostProcessing::Request>;
using pp_rep_msg = std::shared_ptr<depthai_ros_msgs::srv::SetPostProcessing::Response>;
using med_req_msg = const std::shared_ptr<depthai_ros_msgs::srv::SetMedian::Request>;
using med_rep_msg = std::shared_ptr<depthai_ros_msgs::srv::SetMedian::Response>;
using spk_req_msg = const std::shared_ptr<depthai_ros_msgs::srv::SetSpeckle::Request>;
using spk_rep_msg = std::shared_ptr<depthai_ros_msgs::srv::SetSpeckle::Response>;
using tmp_req_msg = const std::shared_ptr<depthai_ros_msgs::srv::SetTemporal::Request>;
using tmp_rep_msg = std::shared_ptr<depthai_ros_msgs::srv::SetTemporal::Response>;
using spt_req_msg = const std::shared_ptr<depthai_ros_msgs::srv::SetSpatial::Request>;
using spt_rep_msg = std::shared_ptr<depthai_ros_msgs::srv::SetSpatial::Response>;
using trh_req_msg = const std::shared_ptr<depthai_ros_msgs::srv::SetThreshold::Request>;
using trh_rep_msg = std::shared_ptr<depthai_ros_msgs::srv::SetThreshold::Response>;
using dcm_req_msg = const std::shared_ptr<depthai_ros_msgs::srv::SetDecimation::Request>;
using dcm_rep_msg = std::shared_ptr<depthai_ros_msgs::srv::SetDecimation::Response>;
using ros_node = std::shared_ptr<rclcpp::Node>;
    #define req_get(x) ((*request).x)
    #define rep_get(x) ((*response).x)
    #define set_parameter(a, b) setRosParameter(node, a, b)
#endif

class DepthPostProcessing {
    using TemporalMode = dai::RawStereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode;
    using DecimationMode = dai::RawStereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode;

   public:
    // DepthPostProcessing();
    DepthPostProcessing(ros_node node, std::string camName);
    void setDevice(std::shared_ptr<dai::Device> device);
    void setFilters();
    dai::RawStereoDepthConfig getFilters(dai::RawStereoDepthConfig config);
    void setConfig(dai::RawStereoDepthConfig config);

    req_type setPostProcessingRequest(pp_req_msg request, pp_rep_msg response);
    req_type setMedianRequest(med_req_msg request, med_rep_msg response);
    req_type setSpeckleRequest(spk_req_msg request, spk_rep_msg response);
    req_type setTemporalRequest(tmp_req_msg request, tmp_rep_msg response);
    req_type setSpatialRequest(spt_req_msg request, spt_rep_msg response);
    req_type setThresholdRequest(trh_req_msg request, trh_rep_msg response);
    req_type setDecimationRequest(dcm_req_msg request, dcm_rep_msg response);

   private:
    dai::MedianFilter getMedianFilter();
    DecimationMode getDecimationMode();
    TemporalMode getTemporalMode();
    bool _median_enable = false;
    std::string _median_mode = "MEDIAN_OFF";
    bool _speckle_enable = false;
    int _speckle_range = 50;
    bool _temporal_enable = false;
    std::string _temporal_mode = "PERSISTENCY_OFF";
    float _temporal_alpha = 0.4;
    int _temporal_delta = 0;
    bool _spatial_enable = false;
    int _spatial_radius = 2;
    float _spatial_alpha = 0.5;
    int _spatial_delta = 0;
    int _spatial_iterations = 1;
    bool _threshold_enable = false;
    int _threshold_max = 0;
    int _threshold_min = 0;
    bool _decimation_enable = false;
    std::string _decimation_mode = "NON_ZERO_MEDIAN";
    int _decimation_factor = 1;
    std::shared_ptr<dai::Device> _device;
    dai::RawStereoDepthConfig _config;
};
