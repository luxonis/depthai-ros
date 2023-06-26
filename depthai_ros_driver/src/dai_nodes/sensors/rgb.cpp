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
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
RGB::RGB(const std::string& daiNodeName,
         rclcpp::Node* node,
         std::shared_ptr<dai::Pipeline> pipeline,
         dai::CameraBoardSocket socket = dai::CameraBoardSocket::RGB,
         sensor_helpers::ImageSensor sensor = {"IMX378", {"12mp", "4k"}, true},
         bool publish = true)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    colorCamNode = pipeline->create<dai::node::ColorCamera>();
    ph = std::make_unique<param_handlers::SensorParamHandler>(node, daiNodeName);
    ph->declareParams(colorCamNode, socket, sensor, publish);
    setXinXout(pipeline);
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
}
RGB::~RGB() = default;
void RGB::setNames() {
    ispQName = getName() + "_isp";
    previewQName = getName() + "_preview";
    controlQName = getName() + "_control";
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
    xinControl = pipeline->create<dai::node::XLinkIn>();
    xinControl->setStreamName(controlQName);
    xinControl->out.link(colorCamNode->inputControl);
}

void RGB::setupQueues(std::shared_ptr<dai::Device> device) {
    if(ph->getParam<bool>("i_publish_topic")) {
        auto tfPrefix = getTFPrefix(getName());
        infoManager = std::make_shared<camera_info_manager::CameraInfoManager>(
            getROSNode()->create_sub_node(std::string(getROSNode()->get_name()) + "/" + getName()).get(), "/" + getName());
        imageConverter =
            std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false, ph->getParam<bool>("i_get_base_device_timestamp"));
        imageConverter->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
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
        rgbPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");
        colorQ = device->getOutputQueue(ispQName, ph->getParam<int>("i_max_q_size"), false);
        if(ph->getParam<bool>("i_low_bandwidth")) {
            colorQ->addCallback(std::bind(sensor_helpers::compressedImgCB,
                                          std::placeholders::_1,
                                          std::placeholders::_2,
                                          *imageConverter,
                                          rgbPub,
                                          infoManager,
                                          dai::RawImgFrame::Type::BGR888i));
        } else {
            colorQ->addCallback(std::bind(sensor_helpers::imgCB, std::placeholders::_1, std::placeholders::_2, *imageConverter, rgbPub, infoManager));
        }
    }
    if(ph->getParam<bool>("i_enable_preview")) {
        previewQ = device->getOutputQueue(previewQName, ph->getParam<int>("i_max_q_size"), false);
        previewPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/preview/image_raw");
        previewInfoManager = std::make_shared<camera_info_manager::CameraInfoManager>(
            getROSNode()->create_sub_node(std::string(getROSNode()->get_name()) + "/" + previewQName).get(), previewQName);
        auto tfPrefix = getTFPrefix(getName());
        imageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
        imageConverter->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
        if(ph->getParam<std::string>("i_calibration_file").empty()) {
            previewInfoManager->setCameraInfo(sensor_helpers::getCalibInfo(getROSNode()->get_logger(),
                                                                           *imageConverter,
                                                                           device,
                                                                           static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id")),
                                                                           ph->getParam<int>("i_preview_size"),
                                                                           ph->getParam<int>("i_preview_size")));
        } else {
            infoManager->loadCameraInfo(ph->getParam<std::string>("i_calibration_file"));
        }
        previewQ->addCallback(std::bind(sensor_helpers::imgCB, std::placeholders::_1, std::placeholders::_2, *imageConverter, previewPub, previewInfoManager));
    };
    controlQ = device->getInputQueue(controlQName);
}

void RGB::closeQueues() {
    if(ph->getParam<bool>("i_publish_topic")) {
        colorQ->close();
        if(ph->getParam<bool>("i_enable_preview")) {
            previewQ->close();
        }
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
