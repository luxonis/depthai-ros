#include "depthai_ros_driver/dai_nodes/sensors/vio.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai_bridge/TransformDataConverter.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/imu.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/stereo.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "depthai_ros_driver/param_handlers/vio_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/node.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace depthai_ros_driver {
namespace dai_nodes {
Vio::Vio(const std::string& daiNodeName,
         std::shared_ptr<rclcpp::Node> node,
         std::shared_ptr<dai::Pipeline> pipeline,
         std::shared_ptr<dai::Device> device,
         bool rsCompat,
         SensorWrapper& left,
         SensorWrapper& right,
         Imu& imu)
    : BaseNode(daiNodeName, node, pipeline, device->getDeviceName(), rsCompat) {
    using namespace param_handlers;
    RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    vioNode = pipeline->create<dai::node::BasaltVIO>();
    ph = std::make_unique<VioParamHandler>(node, daiNodeName, device->getDeviceName(), rsCompat);
    ph->declareParams(vioNode);
    frameId = ph->getParam<std::string>("i_frame_id");
    childFrameId = ph->getParam<std::string>("i_child_frame_id");
    auto width = ph->getParam<int>(ParamNames::WIDTH);
    auto height = ph->getParam<int>(ParamNames::HEIGHT);
    auto fps = ph->getParam<double>(ParamNames::FPS);
    socket = ph->getSocketID();
    imu.link(vioNode->imu);

    left.getUnderlyingNode()->requestOutput(std::make_pair(width, height), std::nullopt, dai::ImgResizeMode::CROP, fps)->link(vioNode->left);
    right.getUnderlyingNode()->requestOutput(std::make_pair(width, height), std::nullopt, dai::ImgResizeMode::CROP, fps)->link(vioNode->right);
    vioNode->setImuUpdateRate(ph->getParam<int>("i_imu_update_rate"));
    publishTf = ph->getParam<bool>("i_publish_tf");
    if(publishTf) {
        tfBr = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    }

    RCLCPP_DEBUG(getLogger(), "Node %s created", daiNodeName.c_str());
}
Vio::Vio(const std::string& daiNodeName,
         std::shared_ptr<rclcpp::Node> node,
         std::shared_ptr<dai::Pipeline> pipeline,
         std::shared_ptr<dai::Device> device,
         bool rsCompat,
         Stereo& stereo,
         Imu& imu)
    : BaseNode(daiNodeName, node, pipeline, device->getDeviceName(), rsCompat) {
    using namespace param_handlers;
    RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    vioNode = pipeline->create<dai::node::BasaltVIO>();
    ph = std::make_unique<VioParamHandler>(node, daiNodeName, device->getDeviceName(), rsCompat);
    ph->declareParams(vioNode);
    frameId = ph->getParam<std::string>("i_frame_id");
    childFrameId = ph->getParam<std::string>("i_child_frame_id");
    imu.link(vioNode->imu);
    auto width = ph->getParam<int>(ParamNames::WIDTH);
    auto height = ph->getParam<int>(ParamNames::HEIGHT);
    auto fps = ph->getParam<double>(ParamNames::FPS);
    socket = ph->getSocketID();
    stereo.getLeftSensor()->getUnderlyingNode()->requestOutput(std::make_pair(width, height), std::nullopt, dai::ImgResizeMode::CROP, fps)->link(vioNode->left);
    stereo.getRightSensor()
        ->getUnderlyingNode()
        ->requestOutput(std::make_pair(width, height), std::nullopt, dai::ImgResizeMode::CROP, fps)
        ->link(vioNode->right);
    vioNode->setImuUpdateRate(ph->getParam<int>("i_imu_update_rate"));
    publishTf = ph->getParam<bool>("i_publish_tf");
    if(publishTf) {
        tfBr = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    }

    RCLCPP_DEBUG(getLogger(), "Node %s created", daiNodeName.c_str());
}
Vio::~Vio() = default;

void Vio::setNames() {}

void Vio::setInOut(std::shared_ptr<dai::Pipeline> /* pipeline */) {}

void Vio::setupQueues(std::shared_ptr<dai::Device> /* device */) {
    using ParamNames = param_handlers::ParamNames;
    transQ = vioNode->transform.createOutputQueue(ph->getParam<int>(ParamNames::MAX_Q_SIZE), false);
    auto tfPrefix = frameId;
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = rclcpp::QosOverridingOptions();
    odomConv = std::make_unique<depthai_bridge::TransformDataConverter>(tfPrefix, childFrameId, ph->getParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP));
    odomConv->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG));
    odomConv->setCovariance(ph->getParam<std::vector<double>>("i_covariance"));

    odomPub = getROSNode()->create_publisher<nav_msgs::msg::Odometry>("~/" + getName() + "/odometry", ph->getParam<int>(ParamNames::MAX_Q_SIZE), options);
    transQ->addCallback(std::bind(&Vio::transCB, this, std::placeholders::_1, std::placeholders::_2));
}

void Vio::closeQueues() {
    transQ->close();
}

void Vio::transCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto transData = std::dynamic_pointer_cast<dai::TransformData>(data);
    std::deque<nav_msgs::msg::Odometry> deq;
    odomConv->toRosMsg(transData, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        currMsg.header.stamp = getROSNode()->get_clock()->now();
        odomPub->publish(currMsg);
        if(publishTf) {
            geometry_msgs::msg::TransformStamped transformMsg;
            transformMsg.header.stamp = currMsg.header.stamp;
            transformMsg.header.frame_id = frameId;
            transformMsg.child_frame_id = childFrameId;
            transformMsg.transform.translation.x = currMsg.pose.pose.position.x;
            transformMsg.transform.translation.y = currMsg.pose.pose.position.y;
            transformMsg.transform.translation.z = currMsg.pose.pose.position.z;
            transformMsg.transform.rotation = currMsg.pose.pose.orientation;
            tfBr->sendTransform(transformMsg);
        }
        deq.pop_front();
    }
}

void Vio::link(dai::Node::Input& in, int /*linkType*/) {
    vioNode->transform.link(in);
}

dai::Node::Input& Vio::getInput(int linkType) {
    if(linkType == static_cast<int>(link_types::VioLinkType::left)) {
        return vioNode->left;
    } else if(linkType == static_cast<int>(link_types::VioLinkType::right)) {
        return vioNode->right;
    } else if(linkType == static_cast<int>(link_types::VioLinkType::imu)) {
        return vioNode->imu;
    } else {
        RCLCPP_ERROR(getLogger(), "Wrong link type: %d", linkType);
        throw std::runtime_error("Wrong link type specified!");
    }
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
