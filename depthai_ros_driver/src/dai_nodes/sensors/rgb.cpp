#include "depthai_ros_driver/dai_nodes/sensors/rgb.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_bridge/ImageConverter.hpp"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"

namespace depthai_ros_driver {
namespace dai_nodes {
RGB::RGB(const std::string& daiNodeName,
         ros::NodeHandle node,
         std::shared_ptr<dai::Pipeline> pipeline,
         dai::CameraBoardSocket socket = dai::CameraBoardSocket::RGB,
         sensor_helpers::ImageSensor sensor = {"IMX378", {"12mp", "4k"}, true},
         bool publish = true)
    : BaseNode(daiNodeName, node, pipeline), it(node) {
    ROS_DEBUG("Creating node %s", daiNodeName.c_str());
    setNames();
    colorCamNode = pipeline->create<dai::node::ColorCamera>();
    ph = std::make_unique<param_handlers::RGBParamHandler>(daiNodeName);
    ph->declareParams(node, colorCamNode, socket, sensor, publish);

    setXinXout(pipeline);
    ROS_DEBUG("Node %s created", daiNodeName.c_str());
}
void RGB::setNames() {
    ispQName = getName() + "_isp";
    previewQName = getName() + "_preview";
    controlQName = getName() + "_control";
}

void RGB::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    if(ph->getParam<bool>(getROSNode(), "i_publish_topic")) {
        xoutColor = pipeline->create<dai::node::XLinkOut>();
        xoutColor->setStreamName(ispQName);
        if(ph->getParam<bool>(getROSNode(), "i_low_bandwidth")) {
            videoEnc = sensor_helpers::createEncoder(pipeline, ph->getParam<int>(getROSNode(), "i_low_bandwidth_quality"));
            colorCamNode->video.link(videoEnc->input);
            videoEnc->bitstream.link(xoutColor->input);
        } else {
            colorCamNode->isp.link(xoutColor->input);
        }
    }
    if(ph->getParam<bool>(getROSNode(), "i_enable_preview")) {
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
    if(ph->getParam<bool>(getROSNode(), "i_publish_topic")) {
        auto tfPrefix = getTFPrefix(getName());
        infoManager = std::make_shared<camera_info_manager::CameraInfoManager>(ros::NodeHandle(getROSNode(), getName()), "/" + getName());
        imageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
        if(ph->getParam<std::string>(getROSNode(), "i_calibration_file").empty()) {
            infoManager->setCameraInfo(sensor_helpers::getCalibInfo(*imageConverter,
                                                                    device,
                                                                    static_cast<dai::CameraBoardSocket>(ph->getParam<int>(getROSNode(), "i_board_socket_id")),
                                                                    ph->getParam<int>(getROSNode(), "i_width"),
                                                                    ph->getParam<int>(getROSNode(), "i_height")));
        } else {
            infoManager->loadCameraInfo(ph->getParam<std::string>(getROSNode(), "i_calibration_file"));
        }
        rgbPub = it.advertiseCamera(getName() + "/image_raw", 1);
        colorQ = device->getOutputQueue(ispQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
        if(ph->getParam<bool>(getROSNode(), "i_low_bandwidth")) {
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
    if(ph->getParam<bool>(getROSNode(), "i_enable_preview")) {
        previewQ = device->getOutputQueue(previewQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
        previewPub = it.advertiseCamera(getName() + "/preview/image_raw", 1);
        previewInfoManager = std::make_shared<camera_info_manager::CameraInfoManager>(ros::NodeHandle(getROSNode(), "/" + previewQName), previewQName);
        auto tfPrefix = getTFPrefix(getName());
        imageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
        if(ph->getParam<std::string>(getROSNode(), "i_calibration_file").empty()) {
            previewInfoManager->setCameraInfo(
                sensor_helpers::getCalibInfo(*imageConverter,
                                             device,
                                             static_cast<dai::CameraBoardSocket>(ph->getParam<int>(getROSNode(), "i_board_socket_id")),
                                             ph->getParam<int>(getROSNode(), "i_preview_size"),
                                             ph->getParam<int>(getROSNode(), "i_preview_size")));
        } else {
            infoManager->loadCameraInfo(ph->getParam<std::string>(getROSNode(), "i_calibration_file"));
        }
        previewQ->addCallback(std::bind(sensor_helpers::imgCB, std::placeholders::_1, std::placeholders::_2, *imageConverter, previewPub, previewInfoManager));
    };
    controlQ = device->getInputQueue(controlQName);
}

void RGB::closeQueues() {
    if(ph->getParam<bool>(getROSNode(), "i_publish_topic")) {
        colorQ->close();
        if(ph->getParam<bool>(getROSNode(), "i_enable_preview")) {
            previewQ->close();
        }
    }
    controlQ->close();
}

void RGB::link(const dai::Node::Input& in, int linkType) {
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
    auto ctrl = ph->setRuntimeParams(getROSNode(), config);
    controlQ->send(ctrl);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
