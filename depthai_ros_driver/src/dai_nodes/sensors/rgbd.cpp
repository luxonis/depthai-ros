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
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
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
           std::shared_ptr<dai::node::StereoDepth> stereo,
           bool aligned)
    : BaseNode(daiNodeName, node, pipeline, device->getDeviceName(), rsCompat) {
    using namespace param_handlers;
    RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    rgbdNode = pipeline->create<dai::node::RGBD>()->build();
    ph = std::make_unique<RGBDParamHandler>(node, daiNodeName, device->getDeviceName(), rsCompat);
    ph->declareParams(rgbdNode, camNode.getSocketID());
    int threadNum = ph->getParam<int>("i_num_threads");
    if(threadNum > 1) {
        rgbdNode->useCPUMT(threadNum);
    }
    auto color = camNode.getUnderlyingNode();
    auto platform = device->getPlatform();
    rgbdNode->runSyncOnHost(ph->getParam<bool>("i_run_sync_on_host"));
    auto fps = ph->getOtherNodeParam<float>(camNode.getName(), ParamNames::FPS);

    auto* out = color->requestOutput(std::pair<int, int>(ph->getOtherNodeParam<int>(camNode.getName(), ParamNames::WIDTH),
                                                         ph->getOtherNodeParam<int>(camNode.getName(), ParamNames::HEIGHT)),
                                     dai::ImgFrame::Type::RGB888i,
                                     dai::ImgResizeMode::CROP,
                                     fps,
                                     true);
    out->link(rgbdNode->inColor);
    if(platform == dai::Platform::RVC4) {
        if(!aligned) {
            align = pipeline->create<dai::node::ImageAlign>();
            align->setRunOnHost(ph->getParam<bool>("i_run_align_on_host"));
            stereo->depth.link(align->input);
            out->link(align->inputAlignTo);
            align->inputAlignTo.setBlocking(false);
            align->input.setBlocking(false);
            align->outputAligned.link(rgbdNode->inDepth);
        } else {
            RCLCPP_DEBUG(getLogger(), "Stereo depth output is reported to be aligned. Please connect its output externally");
        }
    } else {
        if(!aligned) {
            out->link(stereo->inputAlignTo);
            stereo->inputAlignTo.setBlocking(false);
        }
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
           ToF& tofNode,
           bool aligned)
    : BaseNode(daiNodeName, node, pipeline, device->getDeviceName(), rsCompat) {
    using namespace param_handlers;
    RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    rgbdNode = pipeline->create<dai::node::RGBD>()->build();
    ph = std::make_unique<RGBDParamHandler>(node, daiNodeName, device->getDeviceName(), rsCompat);
    ph->declareParams(rgbdNode, camNode.getSocketID());
    auto color = camNode.getUnderlyingNode();
    auto tof = tofNode.getUnderlyingNode();
    auto fps = ph->getOtherNodeParam<float>(camNode.getName(), ParamNames::FPS);
    auto* out = color->requestOutput(std::pair<int, int>(ph->getOtherNodeParam<int>(camNode.getName(), ParamNames::WIDTH),
                                                         ph->getOtherNodeParam<int>(camNode.getName(), ParamNames::HEIGHT)),
                                     dai::ImgFrame::Type::RGB888i,
                                     dai::ImgResizeMode::CROP,
                                     fps,
                                     true);
    out->link(rgbdNode->inColor);
    if(!aligned) {
        align = pipeline->create<dai::node::ImageAlign>();
        align->setRunOnHost(ph->getParam<bool>("i_run_align_on_host"));
        tof->depth.link(align->input);
        out->link(align->inputAlignTo);
        align->inputAlignTo.setBlocking(false);
        align->input.setBlocking(false);
        align->outputAligned.link(rgbdNode->inDepth);
    } else {
        RCLCPP_DEBUG(getLogger(), "Depth has been prealigned! Please remember to link manually in pipeline creation.");
    }

    RCLCPP_DEBUG(getLogger(), "Node %s created", daiNodeName.c_str());
}
RGBD::~RGBD() = default;

void RGBD::setNames() {}

void RGBD::setInOut(std::shared_ptr<dai::Pipeline> /* pipeline */) {}

void RGBD::setupQueues(std::shared_ptr<dai::Device> /* device */) {
    using ParamNames = param_handlers::ParamNames;
    pclQ = rgbdNode->pcl.createOutputQueue(ph->getParam<int>(ParamNames::MAX_Q_SIZE), false);
    auto tfPrefix = getOpticalFrameName(getSocketName(ph->getSocketID()));
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = rclcpp::QosOverridingOptions();
    pclConv = std::make_unique<depthai_bridge::PointCloudConverter>(tfPrefix, ph->getParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP));
    pclConv->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG));
    pclConv->setDepthUnit(dai::StereoDepthConfig::AlgorithmControl::DepthUnit::METER);

    pclPub = getROSNode()->create_publisher<sensor_msgs::msg::PointCloud2>("~/" + getName() + "/points", ph->getParam<int>(ParamNames::MAX_Q_SIZE), options);
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

void RGBD::link(dai::Node::Input& in, int /*linkType*/) {
    rgbdNode->pcl.link(in);
}

dai::Node::Input& RGBD::getInput(int linkType) {
    if(linkType == static_cast<int>(link_types::RGBDLinkType::rgb)) {
        return rgbdNode->inColor;
    } else if(linkType == static_cast<int>(link_types::RGBDLinkType::depth)) {
        return rgbdNode->inDepth;
    } else {
        RCLCPP_ERROR(getLogger(), "Wrong link type: %d", linkType);
        throw std::runtime_error("Wrong link type specified!");
    }
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
