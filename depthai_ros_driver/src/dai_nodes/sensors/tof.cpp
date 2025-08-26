#include "depthai_ros_driver/dai_nodes/sensors/tof.hpp"

#include <depthai/common/CameraBoardSocket.hpp>
#include <stdexcept>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ImageAlign.hpp"
#include "depthai/pipeline/node/ToF.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "depthai_ros_driver/param_handlers/tof_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
ToF::ToF(const std::string& daiNodeName,
         std::shared_ptr<rclcpp::Node> node,
         std::shared_ptr<dai::Pipeline> pipeline,
         const std::string& deviceName,
         bool rsCompat,
         dai::CameraBoardSocket socket)
    : BaseNode(daiNodeName, node, pipeline, deviceName, rsCompat) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    tofNode = pipeline->create<dai::node::ToF>();
    boardSocket = socket;
    ph = std::make_unique<param_handlers::ToFParamHandler>(node, daiNodeName, deviceName, rsCompat);
    ph->declareParams(tofNode, socket);
    setInOut(pipeline);
    aligned = ph->getParam<bool>(param_handlers::ParamNames::ALIGNED);
    if(aligned) {
        alignNode = pipeline->create<dai::node::ImageAlign>();
        alignNode->setRunOnHost(ph->getParam<bool>("i_run_align_on_host"));
        RCLCPP_DEBUG(getLogger(), "ToF is aligned, make sure to connect inputs/outputs in pipeline creation");
    }
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
}
ToF::~ToF() = default;
void ToF::setNames() {
    tofQName = getName() + "_tof";
}
std::shared_ptr<dai::node::ToF> ToF::getUnderlyingNode() {
    return tofNode;
}

void ToF::setInOut(std::shared_ptr<dai::Pipeline> pipeline) {
    using param_handlers::ParamNames;
    if(ph->getParam<bool>(ParamNames::PUBLISH_TOPIC)) {
        utils::VideoEncoderConfig encConfig;
        encConfig.profile = static_cast<dai::VideoEncoderProperties::Profile>(ph->getParam<int>(ParamNames::LOW_BANDWIDTH_PROFILE));
        encConfig.bitrate = ph->getParam<int>(ParamNames::LOW_BANDWIDTH_BITRATE);
        encConfig.frameFreq = ph->getParam<int>(ParamNames::LOW_BANDWIDTH_FRAME_FREQ);
        encConfig.quality = ph->getParam<int>(ParamNames::LOW_BANDWIDTH_QUALITY);
        encConfig.enabled = ph->getParam<bool>(ParamNames::LOW_BANDWIDTH);

        tofPub = setupOutput(pipeline, tofQName, &tofNode->depth, ph->getParam<bool>(ParamNames::SYNCED), encConfig);
    }
}

void ToF::setupQueues(std::shared_ptr<dai::Device> device) {
    using param_handlers::ParamNames;
    if(ph->getParam<bool>(ParamNames::PUBLISH_TOPIC)) {
        auto tfPrefix = getOpticalFrameName(getSocketName(boardSocket));

        utils::ImgConverterConfig convConfig;
        convConfig.tfPrefix = tfPrefix;
        convConfig.getBaseDeviceTimestamp = ph->getParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP);
        convConfig.updateROSBaseTimeOnRosMsg = ph->getParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG);
        convConfig.lowBandwidth = ph->getParam<bool>(ParamNames::LOW_BANDWIDTH);
        convConfig.encoding = dai::ImgFrame::Type::RAW8;
        convConfig.addExposureOffset = ph->getParam<bool>(ParamNames::ADD_EXPOSURE_OFFSET);
        convConfig.expOffset = static_cast<dai::CameraExposureOffset>(ph->getParam<int>(ParamNames::EXPOSURE_OFFSET));
        convConfig.reverseSocketOrder = ph->getParam<bool>(ParamNames::REVERSE_STEREO_SOCKET_ORDER);

        utils::ImgPublisherConfig pubConfig;
        pubConfig.daiNodeName = getName();
        pubConfig.topicName = "~/" + getName();
        pubConfig.lazyPub = ph->getParam<bool>(ParamNames::ENABLE_LAZY_PUBLISHER);
        pubConfig.socket = ph->getSocketID();
        pubConfig.calibrationFile = ph->getParam<std::string>(ParamNames::CALIBRATION_FILE);
        pubConfig.rectified = false;
        pubConfig.width = ph->getParam<int>(ParamNames::WIDTH);
        pubConfig.height = ph->getParam<int>(ParamNames::HEIGHT);
        pubConfig.maxQSize = ph->getParam<int>(ParamNames::MAX_Q_SIZE);

        tofPub->setup(device, convConfig, pubConfig);
    }
}
void ToF::closeQueues() {
    if(ph->getParam<bool>(param_handlers::ParamNames::PUBLISH_TOPIC)) {
        tofPub->closeQueue();
    }
}

void ToF::link(dai::Node::Input& in, int /*linkType*/) {
    if(aligned) {
        alignNode->outputAligned.link(in);
    } else {
        tofNode->depth.link(in);
    }
}

dai::Node::Input& ToF::getInput(int linkType) {
    if(!aligned) {
        throw std::runtime_error("ToF node is not aligned! Please make sure to enable i_aligned parameter");
    } else {
        return alignNode->inputAlignTo;
    }
}

dai::CameraBoardSocket ToF::getSocketID() {
    return ph->getSocketID();
}

bool ToF::isAligned() {
    return aligned;
}

std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> ToF::getPublishers() {
    std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> pubs;
    using param_handlers::ParamNames;
    if(ph->getParam<bool>(ParamNames::PUBLISH_TOPIC) && ph->getParam<bool>(ParamNames::SYNCED)) {
        pubs.push_back(tofPub);
    }
    return pubs;
}

void ToF::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
