#include "depthai_ros_driver/dai_nodes/sensors/rgbd.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/ImageAlign.hpp"
#include "depthai/pipeline/node/host/RGBD.hpp"
#include "depthai_bridge/PointCloudConverter.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/stereo.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/tof.hpp"
#include "depthai_ros_driver/param_handlers/rgbd_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
RGBD::RGBD(const std::string& daiNodeName,
           std::shared_ptr<rclcpp::Node> node,
           std::shared_ptr<dai::Pipeline> pipeline,
           std::shared_ptr<dai::Device> device,
           bool rsCompat,
           SensorWrapper& camNode,
           Stereo& stereoNode)
    : BaseNode(daiNodeName, node, pipeline, device->getDeviceName(), rsCompat) {
    RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    rgbdNode = pipeline->create<dai::node::RGBD>()->build();
    ph = std::make_unique<param_handlers::RGBDParamHandler>(node, daiNodeName, device->getDeviceName(), rsCompat);
    ph->declareParams(rgbdNode);
    auto color = camNode.getUnderlyingNode();
    auto stereo = stereoNode.getUnderlyingNode();
    auto platform = device->getPlatform();
    if(platform == dai::Platform::RVC4) {
        auto* out = color->requestOutput(
            std::pair<int, int>(ph->getOtherNodeParam<int>(camNode.getName(), "i_width"), ph->getOtherNodeParam<int>(camNode.getName(), "i_height")),
            dai::ImgFrame::Type::RGB888i,
            dai::ImgResizeMode::CROP,
            ph->getOtherNodeParam<float>(camNode.getName(), "i_fps"),
            true);
        out->link(rgbdNode->inColor);
        align = pipeline->create<dai::node::ImageAlign>();
        stereo->depth.link(align->input);
        out->link(align->inputAlignTo);
        align->outputAligned.link(rgbdNode->inDepth);
    } else {
        auto* out = color->requestOutput(
            std::pair<int, int>(ph->getOtherNodeParam<int>(camNode.getName(), "i_width"), ph->getOtherNodeParam<int>(camNode.getName(), "i_height")),
            dai::ImgFrame::Type::RGB888i,
            dai::ImgResizeMode::CROP,
            ph->getOtherNodeParam<float>(camNode.getName(), "i_fps"),
            true);
        out->link(rgbdNode->inColor);
        out->link(stereo->inputAlignTo);
        stereo->depth.link(rgbdNode->inDepth);
    }

    RCLCPP_DEBUG(getLogger(), "Node %s created", daiNodeName.c_str());
}
RGBD::RGBD(const std::string& daiNodeName,
           std::shared_ptr<rclcpp::Node> node,
           std::shared_ptr<dai::Pipeline> pipeline,
           std::shared_ptr<dai::Device> device,
           bool rsCompat,
           SensorWrapper& camNode,
           ToF& tofNode)
    : BaseNode(daiNodeName, node, pipeline, device->getDeviceName(), rsCompat) {
    RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    rgbdNode = pipeline->create<dai::node::RGBD>()->build();
    ph = std::make_unique<param_handlers::RGBDParamHandler>(node, daiNodeName, device->getDeviceName(), rsCompat);
    ph->declareParams(rgbdNode);
    auto color = camNode.getUnderlyingNode();
    auto tof = tofNode.getUnderlyingNode();
    auto platform = device->getPlatform();
    auto* out = color->requestOutput(
        std::pair<int, int>(ph->getOtherNodeParam<int>(camNode.getName(), "i_width"), ph->getOtherNodeParam<int>(camNode.getName(), "i_height")),
        dai::ImgFrame::Type::RGB888i,
        dai::ImgResizeMode::CROP,
        ph->getOtherNodeParam<float>(camNode.getName(), "i_fps"),
        true);
    out->link(rgbdNode->inColor);
    align = pipeline->create<dai::node::ImageAlign>();
    tof->depth.link(align->input);
    out->link(align->inputAlignTo);
    align->outputAligned.link(rgbdNode->inDepth);

    RCLCPP_DEBUG(getLogger(), "Node %s created", daiNodeName.c_str());
}
RGBD::~RGBD() = default;

void RGBD::setNames() {}

void RGBD::setInOut(std::shared_ptr<dai::Pipeline> pipeline) {}

void RGBD::setupQueues(std::shared_ptr<dai::Device> device) {
    pclQ = rgbdNode->pcl.createOutputQueue(ph->getParam<int>("i_max_q_size"), false);
    auto tfPrefix = getOpticalFrameName(getSocketName(static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"))));
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = rclcpp::QosOverridingOptions();
    pclConv = std::make_unique<depthai_bridge::PointCloudConverter>(tfPrefix, ph->getParam<bool>("i_get_base_device_timestamp"));
    pclConv->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
    pclConv->setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::METER);

    pclPub = getROSNode()->create_publisher<sensor_msgs::msg::PointCloud2>("~/" + getName() + "/points", 10, options);
    pclQ->addCallback(std::bind(&RGBD::pclCB, this, std::placeholders::_1, std::placeholders::_2));
}

void RGBD::closeQueues() {
    pclQ->close();
}

void RGBD::pclCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto pclData = std::dynamic_pointer_cast<dai::PointCloudData>(data);
    std::deque<sensor_msgs::msg::PointCloud2> deq;
    pclConv->toRosMsg(pclData, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        pclPub->publish(currMsg);
        deq.pop_front();
    }
}

void RGBD::link(dai::Node::Input in, int /*linkType*/) {
    rgbdNode->pcl.link(in);
}

dai::Node::Input RGBD::getInput(int linkType) {
    if(linkType == static_cast<int>(link_types::RGBDLinkType::rgb)) {
        return rgbdNode->inColor;
    } else if(linkType == static_cast<int>(link_types::RGBDLinkType::depth)) {
        return rgbdNode->inDepth;
    } else {
        throw std::runtime_error("Wrong link type specified!");
    }
}

void RGBD::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
