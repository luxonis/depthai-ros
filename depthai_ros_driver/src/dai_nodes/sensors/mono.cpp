#include "depthai_ros_driver/dai_nodes/sensors/mono.hpp"

#include "camera_info_manager/camera_info_manager.h"
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
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
    ph = std::make_unique<param_handlers::SensorParamHandler>(node, daiNodeName, socket);
    ph->declareParams(monoCamNode, sensor, publish);
    setXinXout(pipeline);
    ROS_DEBUG("Node %s created", daiNodeName.c_str());
}
Mono::~Mono() = default;
void Mono::setNames() {
    monoQName = getName() + "_mono";
    controlQName = getName() + "_control";
}

void Mono::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    if(ph->getParam<bool>("i_publish_topic")) {
        xoutMono = pipeline->create<dai::node::XLinkOut>();
        xoutMono->setStreamName(monoQName);
        if(ph->getParam<bool>("i_low_bandwidth")) {
            videoEnc = sensor_helpers::createEncoder(pipeline, ph->getParam<int>("i_low_bandwidth_quality"));
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
    if(ph->getParam<bool>("i_publish_topic")) {
        monoQ = device->getOutputQueue(monoQName, ph->getParam<int>("i_max_q_size"), false);
        auto tfPrefix = getTFPrefix(utils::getSocketName(static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"))));
        imageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
        imageConverter->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
        if(ph->getParam<bool>("i_low_bandwidth")) {
            imageConverter->convertFromBitstream(dai::RawImgFrame::Type::GRAY8);
        }
        if(ph->getParam<bool>("i_add_exposure_offset")) {
            auto offset = static_cast<dai::CameraExposureOffset>(ph->getParam<int>("i_exposure_offset"));
            imageConverter->addExposureOffset(offset);
        }
        if(ph->getParam<bool>("i_reverse_stereo_socket_order")) {
            imageConverter->reverseStereoSocketOrder();
        }
        monoPubIT = it.advertiseCamera(getName() + "/image_raw", 1);
        infoManager = std::make_shared<camera_info_manager::CameraInfoManager>(ros::NodeHandle(getROSNode(), getName()), "/" + getName());
        if(ph->getParam<std::string>("i_calibration_file").empty()) {
            infoManager->setCameraInfo(sensor_helpers::getCalibInfo(*imageConverter,
                                                                    device,
                                                                    static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id")),
                                                                    ph->getParam<int>("i_width"),
                                                                    ph->getParam<int>("i_height")));
        } else {
            infoManager->loadCameraInfo(ph->getParam<std::string>("i_calibration_file"));
        }
        monoQ->addCallback(std::bind(sensor_helpers::cameraPub,
                                     std::placeholders::_1,
                                     std::placeholders::_2,
                                     *imageConverter,
                                     monoPubIT,
                                     infoManager,
                                     ph->getParam<bool>("i_enable_lazy_publisher")));
    }
    controlQ = device->getInputQueue(controlQName);
}
void Mono::closeQueues() {
    if(ph->getParam<bool>("i_publish_topic")) {
        monoQ->close();
    }
    controlQ->close();
}

void Mono::link(dai::Node::Input in, int /*linkType*/) {
    monoCamNode->out.link(in);
}

void Mono::updateParams(parametersConfig& config) {
    auto ctrl = ph->setRuntimeParams(config);
    controlQ->send(ctrl);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
