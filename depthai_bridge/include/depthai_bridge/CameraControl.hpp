#ifndef CAMERA_CONTROL_HPP_
#define CAMERA_CONTROL_HPP_

#include <iostream>
#include <memory>
#include <string>

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/DisparityConverter.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include "depthai/depthai.hpp"
#include "depthai_bridge/RosParameters.hpp"

#ifndef IS_ROS2

#include "depthai_ros_msgs/SetFocus.h"
#include "depthai_ros_msgs/SetExposure.h"
#include "depthai_ros_msgs/SetWhiteBalance.h"

#include "ros/ros.h"

using exp_req_msg = depthai_ros_msgs::SetExposure::Request&;
using exp_rep_msg = depthai_ros_msgs::SetExposure::Response&;
using foc_req_msg = depthai_ros_msgs::SetFocus::Request&;
using foc_rep_msg = depthai_ros_msgs::SetFocus::Response&;
using wb_req_msg = depthai_ros_msgs::SetWhiteBalance::Request&;
using wb_rep_msg = depthai_ros_msgs::SetWhiteBalance::Response&;

#else

#include "depthai_ros_msgs/srv/set_focus.hpp"
#include "depthai_ros_msgs/srv/set_exposure.hpp"
#include "depthai_ros_msgs/srv/set_white_balance.hpp"
using exp_req_msg = const std::shared_ptr<depthai_ros_msgs::srv::SetExposure::Request>;
using exp_rep_msg = std::shared_ptr<depthai_ros_msgs::srv::SetExposure::Response>;
using foc_req_msg = const std::shared_ptr<depthai_ros_msgs::srv::SetFocus::Request>;
using foc_rep_msg = std::shared_ptr<depthai_ros_msgs::srv::SetFocus::Response>;
using wb_req_msg = const std::shared_ptr<depthai_ros_msgs::srv::SetWhiteBalance::Request>;
using wb_rep_msg = std::shared_ptr<depthai_ros_msgs::srv::SetWhiteBalance::Response>;
using ros_node = std::shared_ptr<rclcpp::Node>;

#endif

struct ExposureParameters {
    int compensation = 0;
    int time_us = 8333;
    int sensitivity_iso = 100;
    bool auto_exposure = true;
    std::array<int, 4> region = {0, 0, 0, 0};
    std::string name;
};

struct FocusParameters {
    std::string mode = "AUTO";
    std::array<int, 4> region = {0, 0, 0, 0};
    int lens_position = 0;
};

struct WhiteBalanceParameters {
    std::string mode = "AUTO";
    bool lock = false;
    int color_temperature = 6500;
};

class CameraControl {
    public:
    CameraControl();
    CameraControl(ros_node node);
    void setDevice(std::shared_ptr<dai::Device> device);
    // Exposure
    void setExposure();
    req_type setRgbExposureRequest(exp_req_msg request, exp_rep_msg response);
    req_type setStereoExposureRequest(exp_req_msg request, exp_rep_msg response);
    void setRgbExposure(bool value);
    // Focus
    void setFocus();
    req_type setFocusRequest(foc_req_msg request, foc_rep_msg response);
    // White Balance
    void setWhiteBalance();
    req_type setWhiteBalanceRequest(wb_req_msg request, wb_rep_msg response);

    private:
    void set_ros_parameters(ros_node node);
    // Exposure
    ExposureParameters _rgb, _stereo;
    std::shared_ptr<dai::Device> _device;
    bool _exposure_rgb = false;
    void setExposure(ExposureParameters exposure);
    // Focus
    FocusParameters _focus;
    int _lens_position = 0;
    dai::CameraControl::AutoFocusMode getFocusMode();
    // White balance
    dai::CameraControl::AutoWhiteBalanceMode getWhiteBalanceMode();
    std::string _white_balance_mode = "AUTO";
    int _color_temperature_k = 6000;
};

#endif
