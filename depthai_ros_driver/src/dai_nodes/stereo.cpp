#include "depthai_ros_driver/dai_nodes/stereo.hpp"

#include "camera_info_manager/camera_info_manager.hpp"
#include "cv_bridge/cv_bridge.h"
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/DeviceBase.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "depthai_ros_driver/param_handlers/stereo_param_handler.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
Stereo::Stereo(const std::string& daiNodeName,
               rclcpp::Node* node,
               std::shared_ptr<dai::Pipeline> pipeline,
               std::shared_ptr<dai::Device> device,
               StereoSensorInfo leftInfo,
               StereoSensorInfo rightInfo)
    : BaseNode(daiNodeName, node, pipeline), leftSensInfo(leftInfo), rightSensInfo(rightInfo) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    stereoCamNode = pipeline->create<dai::node::StereoDepth>();
    left = std::make_unique<SensorWrapper>(leftInfo.name, node, pipeline, device, leftInfo.socket, false);
    right = std::make_unique<SensorWrapper>(rightInfo.name, node, pipeline, device, rightInfo.socket, false);

    ph = std::make_unique<param_handlers::StereoParamHandler>(node, daiNodeName);
    ph->declareParams(stereoCamNode, rightInfo.name);
    setXinXout(pipeline);
    left->link(stereoCamNode->left);
    right->link(stereoCamNode->right);
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
}

Stereo::~Stereo() = default;
void Stereo::setNames() {
    stereoQName = getName() + "_stereo";
    leftRectQName = getName() + "_left_rect";
    rightRectQName = getName() + "_right_rect";
}

void Stereo::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    if(ph->getParam<bool>("i_publish_topic")) {
        xoutStereo = pipeline->create<dai::node::XLinkOut>();
        xoutStereo->setStreamName(stereoQName);
        if(ph->getParam<bool>("i_low_bandwidth")) {
            stereoEnc = sensor_helpers::createEncoder(pipeline, ph->getParam<int>("i_low_bandwidth_quality"));
            stereoCamNode->disparity.link(stereoEnc->input);
            stereoEnc->bitstream.link(xoutStereo->input);
        } else {
            if(ph->getParam<bool>("i_output_disparity")) {
                stereoCamNode->disparity.link(xoutStereo->input);
            } else {
                stereoCamNode->depth.link(xoutStereo->input);
            }
        }
    }
    if(ph->getParam<bool>("i_publish_left_rect")) {
        xoutLeftRect = pipeline->create<dai::node::XLinkOut>();
        xoutLeftRect->setStreamName(leftRectQName);
        if(ph->getParam<bool>("i_left_rect_low_bandwidth")) {
            leftRectEnc = sensor_helpers::createEncoder(pipeline, ph->getParam<int>("i_left_rect_low_bandwidth_quality"));
            stereoCamNode->rectifiedLeft.link(leftRectEnc->input);
            leftRectEnc->bitstream.link(xoutLeftRect->input);
        } else {
            stereoCamNode->rectifiedLeft.link(xoutLeftRect->input);
        }
    }

    if(ph->getParam<bool>("i_publish_right_rect")) {
        xoutRightRect = pipeline->create<dai::node::XLinkOut>();
        xoutRightRect->setStreamName(rightRectQName);
        if(ph->getParam<bool>("i_right_rect_low_bandwidth")) {
            rightRectEnc = sensor_helpers::createEncoder(pipeline, ph->getParam<int>("i_right_rect_low_bandwidth_quality"));
            stereoCamNode->rectifiedRight.link(rightRectEnc->input);
            rightRectEnc->bitstream.link(xoutRightRect->input);
        } else {
            stereoCamNode->rectifiedRight.link(xoutRightRect->input);
        }
    }
}

void Stereo::setupLeftRectQueue(std::shared_ptr<dai::Device> device) {
    auto tfPrefix = getTFPrefix(leftSensInfo.name);
    leftRectConv = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false, ph->getParam<bool>("i_get_base_device_timestamp"));
    leftRectConv->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
    leftRectIM = std::make_shared<camera_info_manager::CameraInfoManager>(
        getROSNode()->create_sub_node(std::string(getROSNode()->get_name()) + "/" + leftSensInfo.name).get(), "/rect");
    auto info = sensor_helpers::getCalibInfo(getROSNode()->get_logger(),
                                             *leftRectConv,
                                             device,
                                             leftSensInfo.socket,
                                             ph->getOtherNodeParam<int>(leftSensInfo.name, "i_width"),
                                             ph->getOtherNodeParam<int>(leftSensInfo.name, "i_height"));
    leftRectIM->setCameraInfo(info);
    leftRectQ = device->getOutputQueue(leftRectQName, ph->getOtherNodeParam<int>(leftSensInfo.name, "i_max_q_size"), false);
    if(getROSNode()->get_node_options().use_intra_process_comms()) {
        leftRectPub = getROSNode()->create_publisher<sensor_msgs::msg::Image>("~/" + leftSensInfo.name + "/image_rect", 10);
        leftRectInfoPub = getROSNode()->create_publisher<sensor_msgs::msg::CameraInfo>("~/" + getName() + "/camera_info", 10);
        leftRectQ->addCallback(std::bind(sensor_helpers::imgCBPtr,
                                         std::placeholders::_1,
                                         std::placeholders::_2,
                                         *leftRectConv,
                                         leftRectPub,
                                         leftRectInfoPub,
                                         leftRectIM,
                                         getROSNode(),
                                         ph->getParam<bool>("i_left_rect_low_bandwidth"),
                                         false));
    } else {
        leftRectPubIT = image_transport::create_camera_publisher(getROSNode(), "~/" + leftSensInfo.name + "/image_rect");
        leftRectQ->addCallback(std::bind(sensor_helpers::imgCBIT,
                                         std::placeholders::_1,
                                         std::placeholders::_2,
                                         *leftRectConv,
                                         leftRectPubIT,
                                         leftRectIM,
                                         getROSNode(),
                                         ph->getParam<bool>("i_left_rect_low_bandwidth"),
                                         false));
    }
}

void Stereo::setupRightRectQueue(std::shared_ptr<dai::Device> device) {
    auto tfPrefix = getTFPrefix(rightSensInfo.name);
    rightRectConv = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false, ph->getParam<bool>("i_get_base_device_timestamp"));
    rightRectConv->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
    rightRectIM = std::make_shared<camera_info_manager::CameraInfoManager>(
        getROSNode()->create_sub_node(std::string(getROSNode()->get_name()) + "/" + rightSensInfo.name).get(), "/rect");
    auto info = sensor_helpers::getCalibInfo(getROSNode()->get_logger(),
                                             *rightRectConv,
                                             device,
                                             rightSensInfo.socket,
                                             ph->getOtherNodeParam<int>(rightSensInfo.name, "i_width"),
                                             ph->getOtherNodeParam<int>(rightSensInfo.name, "i_height"));
    rightRectIM->setCameraInfo(info);
    rightRectQ = device->getOutputQueue(rightRectQName, ph->getOtherNodeParam<int>(rightSensInfo.name, "i_max_q_size"), false);
    if(getROSNode()->get_node_options().use_intra_process_comms()) {
        rightRectPub = getROSNode()->create_publisher<sensor_msgs::msg::Image>("~/" + rightSensInfo.name + "/image_rect", 10);
        rightRectInfoPub = getROSNode()->create_publisher<sensor_msgs::msg::CameraInfo>("~/" + getName() + "/camera_info", 10);
        rightRectQ->addCallback(std::bind(sensor_helpers::imgCBPtr,
                                          std::placeholders::_1,
                                          std::placeholders::_2,
                                          *rightRectConv,
                                          rightRectPub,
                                          rightRectInfoPub,
                                          rightRectIM,
                                          getROSNode(),
                                          ph->getParam<bool>("i_right_rect_low_bandwidth"),
                                          false));
    } else {
        rightRectPubIT = image_transport::create_camera_publisher(getROSNode(), "~/" + rightSensInfo.name + "/image_rect");
        rightRectQ->addCallback(std::bind(sensor_helpers::imgCBIT,
                                          std::placeholders::_1,
                                          std::placeholders::_2,
                                          *rightRectConv,
                                          rightRectPubIT,
                                          rightRectIM,
                                          getROSNode(),
                                          ph->getParam<bool>("i_right_rect_low_bandwidth"),
                                          false));
    }
}

void Stereo::setupStereoQueue(std::shared_ptr<dai::Device> device) {
    std::string tfPrefix;
    if(ph->getParam<bool>("i_align_depth")) {
        tfPrefix = getTFPrefix("rgb");
    } else {
        tfPrefix = getTFPrefix(rightSensInfo.name);
    }
    stereoConv = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false, ph->getParam<bool>("i_get_base_device_timestamp"));
    stereoConv->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
    stereoIM = std::make_shared<camera_info_manager::CameraInfoManager>(
        getROSNode()->create_sub_node(std::string(getROSNode()->get_name()) + "/" + getName()).get(), "/" + getName());
    auto info = sensor_helpers::getCalibInfo(getROSNode()->get_logger(),
                                             *stereoConv,
                                             device,
                                             static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id")),
                                             ph->getParam<int>("i_width"),
                                             ph->getParam<int>("i_height"));
    auto calibHandler = device->readCalibration();
    info.p[3] = calibHandler.getBaselineDistance() * 10.0;  // baseline in mm
    for(auto& d : info.d) {
        d = 0.0;
    }
    stereoIM->setCameraInfo(info);
    stereoQ = device->getOutputQueue(stereoQName, ph->getParam<int>("i_max_q_size"), false);
    if(getROSNode()->get_node_options().use_intra_process_comms()) {
        RCLCPP_DEBUG(getROSNode()->get_logger(), "Enabling intra_process communication!");
        stereoPub = getROSNode()->create_publisher<sensor_msgs::msg::Image>("~/" + getName() + "/image_raw", 10);
        stereoInfoPub = getROSNode()->create_publisher<sensor_msgs::msg::CameraInfo>("~/" + getName() + "/camera_info", 10);
        stereoQ->addCallback(std::bind(sensor_helpers::imgCBPtr,
                                       std::placeholders::_1,
                                       std::placeholders::_2,
                                       *stereoConv,
                                       stereoPub,
                                       stereoInfoPub,
                                       stereoIM,
                                       getROSNode(),
                                       ph->getParam<bool>("i_low_bandwidth"),
                                       !ph->getParam<bool>("i_output_disparity")));
    } else {
        stereoPubIT = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");
        stereoQ->addCallback(std::bind(sensor_helpers::imgCBIT,
                                       std::placeholders::_1,
                                       std::placeholders::_2,
                                       *stereoConv,
                                       stereoPubIT,
                                       stereoIM,
                                       getROSNode(),
                                       ph->getParam<bool>("i_low_bandwidth"),
                                       !ph->getParam<bool>("i_output_disparity")));
    }
}

void Stereo::setupQueues(std::shared_ptr<dai::Device> device) {
    left->setupQueues(device);
    right->setupQueues(device);
    if(ph->getParam<bool>("i_publish_topic")) {
        setupStereoQueue(device);
    }
    if(ph->getParam<bool>("i_publish_left_rect")) {
        setupLeftRectQueue(device);
    }
    if(ph->getParam<bool>("i_publish_right_rect")) {
        setupRightRectQueue(device);
    }
}
void Stereo::closeQueues() {
    left->closeQueues();
    right->closeQueues();
    if(ph->getParam<bool>("i_publish_topic")) {
        stereoQ->close();
    }
    if(ph->getParam<bool>("i_publish_left_rect")) {
        leftRectQ->close();
    }
    if(ph->getParam<bool>("i_publish_right_rect")) {
        rightRectQ->close();
    }
}

void Stereo::link(dai::Node::Input in, int linkType) {
    if(linkType == static_cast<int>(link_types::StereoLinkType::depth)) {
        stereoCamNode->depth.link(in);
    } else {
        if(linkType == static_cast<int>(link_types::StereoLinkType::left)) {
            left->link(in);
        } else if(linkType == static_cast<int>(link_types::StereoLinkType::left_isp)) {
            left->link(in, static_cast<int>(link_types::RGBLinkType::isp));
        } else if(linkType == static_cast<int>(link_types::StereoLinkType::left_preview)) {
            left->link(in, static_cast<int>(link_types::RGBLinkType::preview));
        }

        else if(linkType == static_cast<int>(link_types::StereoLinkType::right)) {
            right->link(in);
        } else if(linkType == static_cast<int>(link_types::StereoLinkType::right_isp)) {
            right->link(in, static_cast<int>(link_types::RGBLinkType::isp));
        } else if(linkType == static_cast<int>(link_types::StereoLinkType::right_preview)) {
            right->link(in, static_cast<int>(link_types::RGBLinkType::preview));
        }

        else {
            throw std::runtime_error("Wrong link type specified!");
        }
    }
}

dai::Node::Input Stereo::getInput(int linkType) {
    if(linkType == static_cast<int>(link_types::StereoLinkType::left)) {
        return stereoCamNode->left;
    } else if(linkType == static_cast<int>(link_types::StereoLinkType::right)) {
        return stereoCamNode->right;
    } else {
        throw std::runtime_error("Wrong link type specified!");
    }
}

void Stereo::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
