#include "depthai_ros_driver/dai_nodes/sensors/rgb.hpp"

#include "camera_info_manager/camera_info_manager.h"
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
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"
#include "ros/node_handle.h"

namespace depthai_ros_driver {
namespace dai_nodes {
RGB::RGB(const std::string& daiNodeName,
         ros::NodeHandle node,
         std::shared_ptr<dai::Pipeline> pipeline,
         dai::CameraBoardSocket socket = dai::CameraBoardSocket::CAM_A,
         sensor_helpers::ImageSensor sensor = {"IMX378", "4k", {"12mp", "4k"}, true},
         bool publish = true)
    : BaseNode(daiNodeName, node, pipeline), it(node) {
    ROS_DEBUG("Creating node %s", daiNodeName.c_str());
    setNames();
    colorCamNode = pipeline->create<dai::node::ColorCamera>();
    ph = std::make_unique<param_handlers::SensorParamHandler>(node, daiNodeName, socket);
    ph->declareParams(colorCamNode, sensor, publish);

    setXinXout(pipeline);
    ROS_DEBUG("Node %s created", daiNodeName.c_str());
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
        auto tfPrefix = getTFPrefix(utils::getSocketName(static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"))));
        infoManager = std::make_shared<camera_info_manager::CameraInfoManager>(ros::NodeHandle(getROSNode(), getName()), "/" + getName());
        imageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
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
        if(ph->getParam<std::string>("i_calibration_file").empty()) {
            infoManager->setCameraInfo(sensor_helpers::getCalibInfo(*imageConverter,
                                                                    device,
                                                                    static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id")),
                                                                    ph->getParam<int>("i_width"),
                                                                    ph->getParam<int>("i_height")));
        } else {
            infoManager->loadCameraInfo(ph->getParam<std::string>("i_calibration_file"));
        }
        rgbPubIT = it.advertiseCamera(getName() + "/image_raw", 1);
        colorQ = device->getOutputQueue(ispQName, ph->getParam<int>("i_max_q_size"), false);
        colorQ->addCallback(std::bind(sensor_helpers::cameraPub,
                                      std::placeholders::_1,
                                      std::placeholders::_2,
                                      *imageConverter,
                                      rgbPubIT,
                                      infoManager,
                                      ph->getParam<bool>("i_enable_lazy_publisher")));
    }
    if(ph->getParam<bool>("i_enable_preview")) {
        previewQ = device->getOutputQueue(previewQName, ph->getParam<int>("i_max_q_size"), false);
        previewPubIT = it.advertiseCamera(getName() + "/preview/image_raw", 1);
        previewInfoManager = std::make_shared<camera_info_manager::CameraInfoManager>(ros::NodeHandle(getROSNode(), "/" + previewQName), previewQName);
        auto tfPrefix = getTFPrefix(utils::getSocketName(static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"))));
        imageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
        if(ph->getParam<std::string>("i_calibration_file").empty()) {
            previewInfoManager->setCameraInfo(sensor_helpers::getCalibInfo(*imageConverter,
                                                                           device,
                                                                           static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id")),
                                                                           ph->getParam<int>("i_preview_size"),
                                                                           ph->getParam<int>("i_preview_size")));
        } else {
            infoManager->loadCameraInfo(ph->getParam<std::string>("i_calibration_file"));
        }
        previewQ->addCallback(
            std::bind(sensor_helpers::basicCameraPub, std::placeholders::_1, std::placeholders::_2, *imageConverter, previewPubIT, previewInfoManager));
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

void RGB::updateParams(parametersConfig& config) {
    auto ctrl = ph->setRuntimeParams(config);
    controlQ->send(ctrl);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
