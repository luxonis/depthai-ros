#include "depthai_ros_driver/dai_nodes/sensors/mono.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_bridge/ImageConverter.hpp"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"

namespace depthai_ros_driver {
namespace dai_nodes {
Mono::Mono(const std::string& daiNodeName,
           ros::NodeHandle node,
           std::shared_ptr<dai::Pipeline> pipeline,
           dai::CameraBoardSocket socket,
           dai_nodes::sensor_helpers::ImageSensor sensor,
           bool publish = true)
    : BaseNode(daiNodeName, node, pipeline), it(node) {
    ROS_DEBUG("Creating node %s", daiNodeName.c_str());
    setNames();
    monoCamNode = pipeline->create<dai::node::MonoCamera>();
    ph = std::make_unique<param_handlers::MonoParamHandler>(daiNodeName);
    ph->declareParams(node, monoCamNode, socket, sensor, publish);
    setXinXout(pipeline);
    ROS_DEBUG("Node %s created", daiNodeName.c_str());
}
void Mono::setNames() {
    monoQName = getName() + "_mono";
    controlQName = getName() + "_control";
}

void Mono::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    if(ph->getParam<bool>(getROSNode(), "i_publish_topic")) {
        xoutMono = pipeline->create<dai::node::XLinkOut>();
        xoutMono->setStreamName(monoQName);
        if(ph->getParam<bool>(getROSNode(), "i_low_bandwidth")) {
            videoEnc = sensor_helpers::createEncoder(pipeline, ph->getParam<int>(getROSNode(), "i_low_bandwidth_quality"));
            monoCamNode->out.link(videoEnc->input);
            videoEnc->bitstream.link(xoutMono->input);
        } else {
            monoCamNode->out.link(xoutMono->input);
        }
    }
    xinControl = pipeline->create<dai::node::XLinkIn>();
    xinControl->setStreamName(controlQName);
    xinControl->out.link(monoCamNode->inputControl);
}

void Mono::setupQueues(std::shared_ptr<dai::Device> device) {
    if(ph->getParam<bool>(getROSNode(), "i_publish_topic")) {
        monoQ = device->getOutputQueue(monoQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
        auto tfPrefix = getTFPrefix(getName());
        imageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
        monoPub = it.advertiseCamera(getName() + "/image_raw", 1);
        infoManager = std::make_shared<camera_info_manager::CameraInfoManager>(ros::NodeHandle(getROSNode(), getName()), "/" + getName());
        if(ph->getParam<std::string>(getROSNode(), "i_calibration_file").empty()) {
            infoManager->setCameraInfo(sensor_helpers::getCalibInfo(*imageConverter,
                                                                    device,
                                                                    static_cast<dai::CameraBoardSocket>(ph->getParam<int>(getROSNode(), "i_board_socket_id")),
                                                                    ph->getParam<int>(getROSNode(), "i_width"),
                                                                    ph->getParam<int>(getROSNode(), "i_height")));
        } else {
            infoManager->loadCameraInfo(ph->getParam<std::string>(getROSNode(), "i_calibration_file"));
        }
        if(ph->getParam<bool>(getROSNode(), "i_low_bandwidth")) {
            monoQ->addCallback(std::bind(sensor_helpers::compressedImgCB,
                                         std::placeholders::_1,
                                         std::placeholders::_2,
                                         *imageConverter,
                                         monoPub,
                                         infoManager,
                                         dai::RawImgFrame::Type::GRAY8));
        } else {
            monoQ->addCallback(std::bind(sensor_helpers::imgCB, std::placeholders::_1, std::placeholders::_2, *imageConverter, monoPub, infoManager));
        }
        controlQ = device->getInputQueue(controlQName);
    }
}
void Mono::closeQueues() {
    if(ph->getParam<bool>(getROSNode(), "i_publish_topic")) {
        monoQ->close();
    }
    controlQ->close();
}

void Mono::link(const dai::Node::Input& in, int /*linkType*/) {
    monoCamNode->out.link(in);
}

void Mono::updateParams(parametersConfig& config) {
    auto ctrl = ph->setRuntimeParams(getROSNode(), config);
    controlQ->send(ctrl);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
