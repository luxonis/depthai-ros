#include "depthai_ros_driver/dai_nodes/sensors/rgb.hpp"

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/param_handlers/sensor_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
RGB::RGB(const std::string& daiNodeName,
         rclcpp::Node* node,
         std::shared_ptr<dai::Pipeline> pipeline,
         dai::CameraBoardSocket socket = dai::CameraBoardSocket::CAM_A,
         sensor_helpers::ImageSensor sensor = {"IMX378", "4k", {"12mp", "4k"}, true},
         bool publish = true)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    colorCamNode = pipeline->create<dai::node::ColorCamera>();
    ph = std::make_unique<param_handlers::SensorParamHandler>(node, daiNodeName, socket);
    ph->declareParams(colorCamNode, sensor, publish);
    setXinXout(pipeline);
    RCLCPP_INFO(node->get_logger(), "Node %s created", daiNodeName.c_str());
}
RGB::~RGB() = default;
void RGB::setNames() {
    ispQName = getName() + "_isp";
    previewQName = getName() + "_preview";
    controlQName = getName() + "_control";
    h264QName = getName() + "_h264";
}

void RGB::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    if(ph->getParam<bool>("i_publish_topic")) {
        xoutColor = pipeline->create<dai::node::XLinkOut>();
        xoutColor->setStreamName(ispQName);
        if(ph->getParam<bool>("i_low_bandwidth")) {
            videoEnc = sensor_helpers::createEncoder(pipeline, ph->getParam<int>("i_low_bandwidth_quality"));
            colorCamNode->video.link(videoEnc->input);
            videoEnc->bitstream.link(xoutColor->input);
        } else {
            if(ph->getParam<bool>("i_output_isp"))
                colorCamNode->isp.link(xoutColor->input);
            else
                colorCamNode->video.link(xoutColor->input);
        }
    }
    if(ph->getParam<bool>("i_enable_preview")) {
        xoutPreview = pipeline->create<dai::node::XLinkOut>();
        xoutPreview->setStreamName(previewQName);
        xoutPreview->input.setQueueSize(2);
        xoutPreview->input.setBlocking(false);
        colorCamNode->preview.link(xoutPreview->input);
    }
    if(ph->getParam<bool>("i_enable_h264")) {
        RCLCPP_INFO(getROSNode()->get_logger(), "Setting up h264 output %s (q=%d)", h264QName.c_str(), ph->getParam<int>("i_h264_quality"));
        videoEncH264 = sensor_helpers::createEncoder(pipeline, ph->getParam<int>("i_h264_quality"), dai::VideoEncoderProperties::Profile::H264_HIGH);
        videoEncH264->setKeyframeFrequency(30); // one kf / second @ 30 hz
        colorCamNode->video.link(videoEncH264->input);

        xoutH264 = pipeline->create<dai::node::XLinkOut>();
        xoutH264->setStreamName(h264QName);
        xoutH264->input.setQueueSize(2);
        xoutH264->input.setBlocking(false);
        videoEncH264->out.link(xoutH264->input);
    }
    xinControl = pipeline->create<dai::node::XLinkIn>();
    xinControl->setStreamName(controlQName);
    xinControl->out.link(colorCamNode->inputControl);
}

void RGB::setupQueues(std::shared_ptr<dai::Device> device) {

    auto tfPrefix = getTFPrefix(utils::getSocketName(static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"))));
    
    imageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false, ph->getParam<bool>("i_get_base_device_timestamp"));
    imageConverter->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
    
    if(ph->getParam<bool>("i_low_bandwidth")) {
        imageConverter->convertFromBitstream(dai::RawImgFrame::Type::BGR888i);
    }
    if(ph->getParam<bool>("i_add_exposure_offset")) {
        auto offset = static_cast<dai::CameraExposureOffset>(ph->getParam<int>("i_exposure_offset"));
        imageConverter->addExposureOffset(offset);
    }
    if(ph->getParam<bool>("i_reverse_stereo_socket_order")) {
        imageConverter->reverseStereoSocketOrder();
    }
    if(ph->getParam<bool>("i_publish_topic")) {
        colorQ = device->getOutputQueue(ispQName, ph->getParam<int>("i_max_q_size"), false);
        infoManager = std::make_shared<camera_info_manager::CameraInfoManager>(
            getROSNode()->create_sub_node(std::string(getROSNode()->get_name()) + "/" + getName()).get(), "/" + getName());
        if(ph->getParam<std::string>("i_calibration_file").empty()) {
            infoManager->setCameraInfo(sensor_helpers::getCalibInfo(getROSNode()->get_logger(),
                                                                    *imageConverter,
                                                                    device,
                                                                    static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id")),
                                                                    ph->getParam<int>("i_width"),
                                                                    ph->getParam<int>("i_height")));
        } else {
            infoManager->loadCameraInfo(ph->getParam<std::string>("i_calibration_file"));
        }
        if(ipcEnabled()) {
            rgbPub = getROSNode()->create_publisher<sensor_msgs::msg::Image>("~/" + getName() + "/image_raw", 10);
            rgbInfoPub = getROSNode()->create_publisher<sensor_msgs::msg::CameraInfo>("~/" + getName() + "/camera_info", 10);
            colorQ->addCallback(std::bind(sensor_helpers::splitPub,
                                          std::placeholders::_1,
                                          std::placeholders::_2,
                                          *imageConverter,
                                          rgbPub,
                                          rgbInfoPub,
                                          infoManager,
                                          ph->getParam<bool>("i_enable_lazy_publisher")));

        } else {
            rgbPubIT = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");
            colorQ->addCallback(std::bind(sensor_helpers::cameraPub,
                                          std::placeholders::_1,
                                          std::placeholders::_2,
                                          *imageConverter,
                                          rgbPubIT,
                                          infoManager,
                                          ph->getParam<bool>("i_enable_lazy_publisher")));
        }
    }
    if(ph->getParam<bool>("i_enable_preview")) {
        previewQ = device->getOutputQueue(previewQName, ph->getParam<int>("i_max_q_size"), false);
        previewInfoManager = std::make_shared<camera_info_manager::CameraInfoManager>(
            getROSNode()->create_sub_node(std::string(getROSNode()->get_name()) + "/" + previewQName).get(), previewQName);
        if(ph->getParam<std::string>("i_calibration_file").empty()) {
            previewInfoManager->setCameraInfo(sensor_helpers::getCalibInfo(getROSNode()->get_logger(),
                                                                           *imageConverter,
                                                                           device,
                                                                           static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id")),
                                                                           ph->getParam<int>("i_preview_size"),
                                                                           ph->getParam<int>("i_preview_size")));
        } else {
            previewInfoManager->loadCameraInfo(ph->getParam<std::string>("i_calibration_file"));
        }
        if(ipcEnabled()) {
            previewPubIT = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/preview/image_raw");
            previewQ->addCallback(
                std::bind(sensor_helpers::basicCameraPub, std::placeholders::_1, std::placeholders::_2, *imageConverter, previewPubIT, previewInfoManager));
        } else {
            previewPub = getROSNode()->create_publisher<sensor_msgs::msg::Image>("~/" + getName() + "/preview/image_raw", 10);
            previewInfoPub = getROSNode()->create_publisher<sensor_msgs::msg::CameraInfo>("~/" + getName() + "/preview/camera_info", 10);
            previewQ->addCallback(std::bind(sensor_helpers::splitPub,
                                            std::placeholders::_1,
                                            std::placeholders::_2,
                                            *imageConverter,
                                            previewPub,
                                            previewInfoPub,
                                            previewInfoManager,
                                            ph->getParam<bool>("i_enable_lazy_publisher")));
        }
    };
    if(ph->getParam<bool>("i_enable_h264")) {
        RCLCPP_INFO(getROSNode()->get_logger(), "Setting up h264 queue %s", tfPrefix.c_str());

        h264Q = device->getOutputQueue(h264QName, ph->getParam<int>("i_max_q_size"), false);
        h264Pub = getROSNode()->create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>("~/" + getName() + "/h264", 10);
        h264Q->addCallback(std::bind(sensor_helpers::videoPub,
                                        std::placeholders::_1,
                                        std::placeholders::_2,
                                        *imageConverter,
                                        h264Pub,
                                        ph->getParam<int>("i_width"), ph->getParam<int>("i_height"), //it's be nice to get these from EncodedFrame
                                        ph->getParam<bool>("i_enable_lazy_publisher")));
    }
    controlQ = device->getInputQueue(controlQName);
}

void RGB::closeQueues() {
    if(ph->getParam<bool>("i_publish_topic")) {
        colorQ->close();
    }
    if(ph->getParam<bool>("i_enable_preview")) {
        previewQ->close();
    }
    if(ph->getParam<bool>("i_enable_h264")) {
        h264Q->close();
    }
    controlQ->close();
}

void RGB::link(dai::Node::Input in, int linkType) {
    if(linkType == static_cast<int>(link_types::RGBLinkType::video)) {
        colorCamNode->video.link(in);
    } else if(linkType == static_cast<int>(link_types::RGBLinkType::isp)) {
        colorCamNode->isp.link(in);
    } else if(linkType == static_cast<int>(link_types::RGBLinkType::preview)) {
        colorCamNode->preview.link(in);
    } else {
        throw std::runtime_error("Link type not supported");
    }
}

void RGB::updateParams(const std::vector<rclcpp::Parameter>& params) {
    auto ctrl = ph->setRuntimeParams(params);
    controlQ->send(ctrl);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
