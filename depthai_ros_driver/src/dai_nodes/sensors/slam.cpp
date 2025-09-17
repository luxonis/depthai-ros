#include "depthai_ros_driver/dai_nodes/sensors/slam.hpp"

#include <depthai/common/CameraBoardSocket.hpp>
#include <depthai/rtabmap/RTABMapSLAM.hpp>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/MapData.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai_bridge/GridMapConverter.hpp"
#include "depthai_bridge/PointCloudConverter.hpp"
#include "depthai_bridge/TransformDataConverter.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/stereo.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/vio.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "depthai_ros_driver/param_handlers/slam_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace depthai_ros_driver {
namespace dai_nodes {
Slam::Slam(const std::string& daiNodeName,
           std::shared_ptr<rclcpp::Node> node,
           std::shared_ptr<dai::Pipeline> pipeline,
           std::shared_ptr<dai::Device> device,
           bool rsCompat,
           SensorWrapper& sens,
           Vio& vio,
           Stereo& stereo)
    : BaseNode(daiNodeName, node, pipeline, device->getDeviceName(), rsCompat) {
    using namespace param_handlers;
    RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    slamNode = pipeline->create<dai::node::RTABMapSLAM>();
    ph = std::make_unique<SlamParamHandler>(node, daiNodeName, device->getDeviceName(), rsCompat);
    ph->declareParams(slamNode);
    mapFrame = ph->getParam<std::string>("i_map_frame");
    odomFrame = ph->getParam<std::string>("i_odom_frame");
    sens.link(slamNode->rect);
    vio.link(slamNode->odom);
    stereo.link(slamNode->depth, static_cast<int>(link_types::StereoLinkType::stereo));

    RCLCPP_DEBUG(getLogger(), "Node %s created", daiNodeName.c_str());
}
Slam::~Slam() = default;

void Slam::setNames() {}

void Slam::setInOut(std::shared_ptr<dai::Pipeline> pipeline) {}

void Slam::setupQueues(std::shared_ptr<dai::Device> device) {
    using ParamNames = param_handlers::ParamNames;
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = rclcpp::QosOverridingOptions();
    if(ph->getParam<bool>("i_publish_tf")) {
        tfBr = std::make_shared<tf2_ros::TransformBroadcaster>(getROSNode());
        mapToOdomQ = slamNode->odomCorrection.createOutputQueue(ph->getParam<int>(ParamNames::MAX_Q_SIZE), false);
        mapToOdomConv =
            std::make_unique<depthai_bridge::TransformDataConverter>(mapFrame, odomFrame, ph->getParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP));
        mapToOdomConv->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG));
        mapToOdomQ->addCallback(std::bind(&Slam::mapToOdomCB, this, std::placeholders::_1, std::placeholders::_2));
    }
    if(ph->getParam<bool>("i_publish_absolute_pose")) {
        absolutePoseQ = slamNode->transform.createOutputQueue(ph->getParam<int>(ParamNames::MAX_Q_SIZE), false);
        absolutePosePub = getROSNode()->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "~/" + getName() + "/absolute_pose", ph->getParam<int>(ParamNames::MAX_Q_SIZE), options);
        absolutePoseConv =
            std::make_unique<depthai_bridge::TransformDataConverter>(mapFrame, baseFrame, ph->getParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP));
        absolutePoseConv->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG));
        absolutePoseQ->addCallback(std::bind(&Slam::absolutePoseCB, this, std::placeholders::_1, std::placeholders::_2));
    }
    if(ph->getParam<bool>("i_publish_map")) {
        mapQ = slamNode->occupancyGridMap.createOutputQueue(ph->getParam<int>(ParamNames::MAX_Q_SIZE), false);
        mapConv = std::make_unique<depthai_bridge::GridMapConverter>(mapFrame, ph->getParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP));
        mapConv->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG));
        mapPub = getROSNode()->create_publisher<nav_msgs::msg::OccupancyGrid>("~/" + getName() + "/map", ph->getParam<int>(ParamNames::MAX_Q_SIZE), options);
        mapQ->addCallback(std::bind(&Slam::mapCB, this, std::placeholders::_1, std::placeholders::_2));
    }
    if(ph->getParam<bool>("i_publish_ground_pcl")) {
        groundPclQ = slamNode->groundPCL.createOutputQueue(ph->getParam<int>(ParamNames::MAX_Q_SIZE), false);
        groundPclConv = std::make_unique<depthai_bridge::PointCloudConverter>(mapFrame, ph->getParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP));
        groundPclConv->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG));
        groundPclPub =
            getROSNode()->create_publisher<sensor_msgs::msg::PointCloud2>("~/" + getName() + "/ground_pcl", ph->getParam<int>(ParamNames::MAX_Q_SIZE), options);
        groundPclQ->addCallback(std::bind(&Slam::groundPclCB, this, std::placeholders::_1, std::placeholders::_2));
    }
    if(ph->getParam<bool>("i_publish_obstacle_pcl")) {
        obstaclePclQ = slamNode->obstaclePCL.createOutputQueue(ph->getParam<int>(ParamNames::MAX_Q_SIZE), false);
        obstaclePclConv = std::make_unique<depthai_bridge::PointCloudConverter>(mapFrame, ph->getParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP));
        obstaclePclConv->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG));
        obstaclePclPub = getROSNode()->create_publisher<sensor_msgs::msg::PointCloud2>(
            "~/" + getName() + "/obstacle_pcl", ph->getParam<int>(ParamNames::MAX_Q_SIZE), options);
        obstaclePclQ->addCallback(std::bind(&Slam::obstaclePclCB, this, std::placeholders::_1, std::placeholders::_2));
    }
}

void Slam::closeQueues() {
    if(ph->getParam<bool>("i_publish_tf")) {
        mapToOdomQ->close();
    }
    if(ph->getParam<bool>("i_publish_absolute_pose")) {
        absolutePoseQ->close();
    }
    if(ph->getParam<bool>("i_publish_map")) {
        mapQ->close();
    }
    if(ph->getParam<bool>("i_publish_ground_pcl")) {
        groundPclQ->close();
    }
    if(ph->getParam<bool>("i_publish_obstacle_pcl")) {
        obstaclePclQ->close();
    }
}

void Slam::mapToOdomCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto transData = std::dynamic_pointer_cast<dai::TransformData>(data);
    std::deque<geometry_msgs::msg::TransformStamped> deq;
    mapToOdomConv->toRosMsg(transData, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        // pose timestamp might be too late
        currMsg.header.stamp = getROSNode()->get_clock()->now();
        tfBr->sendTransform(currMsg);
        deq.pop_front();
    }
}

void Slam::absolutePoseCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto transData = std::dynamic_pointer_cast<dai::TransformData>(data);
    std::deque<geometry_msgs::msg::PoseWithCovarianceStamped> deq;
    absolutePoseConv->toRosMsg(transData, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        // pose timestamp might be too late
        currMsg.header.stamp = getROSNode()->get_clock()->now();
        absolutePosePub->publish(currMsg);
        deq.pop_front();
    }
}

void Slam::mapCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto mapData = std::dynamic_pointer_cast<dai::MapData>(data);
    std::deque<nav_msgs::msg::OccupancyGrid> deq;
    mapConv->toRosMsg(mapData, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        mapPub->publish(currMsg);
        deq.pop_front();
    }
}

void Slam::groundPclCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto pclData = std::dynamic_pointer_cast<dai::PointCloudData>(data);
    std::deque<sensor_msgs::msg::PointCloud2> deq;
    groundPclConv->toRosMsg(pclData, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        groundPclPub->publish(currMsg);
        deq.pop_front();
    }
}

void Slam::obstaclePclCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto pclData = std::dynamic_pointer_cast<dai::PointCloudData>(data);
    std::deque<sensor_msgs::msg::PointCloud2> deq;
    obstaclePclConv->toRosMsg(pclData, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        obstaclePclPub->publish(currMsg);
        deq.pop_front();
    }
}

void Slam::link(dai::Node::Input& in, int linkType) {
    if(linkType == static_cast<int>(link_types::SlamLinkType::map)) {
        slamNode->occupancyGridMap.link(in);
    } else if(linkType == static_cast<int>(link_types::SlamLinkType::groundPcl)) {
        slamNode->groundPCL.link(in);
    } else if(linkType == static_cast<int>(link_types::SlamLinkType::obstaclePcl)) {
        slamNode->obstaclePCL.link(in);
    } else if(linkType == static_cast<int>(link_types::SlamLinkType::transform)) {
        slamNode->transform.link(in);
    } else {
        RCLCPP_ERROR(getLogger(), "Wrong link type: %d", linkType);
        throw std::runtime_error("Wrong link type specified!");
    }
}

dai::CameraBoardSocket Slam::getSocketID() {
    return ph->getSocketID();
}

dai::Node::Input& Slam::getInput(int linkType) {
    if(linkType == static_cast<int>(link_types::SlamLinkType::rect)) {
        return slamNode->rect;
    } else if(linkType == static_cast<int>(link_types::SlamLinkType::depth)) {
        return slamNode->depth;
    } else if(linkType == static_cast<int>(link_types::SlamLinkType::transform)) {
        return slamNode->odom;
    } else {
        RCLCPP_ERROR(getLogger(), "Wrong link type: %d", linkType);
        throw std::runtime_error("Wrong link type specified!");
    }
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
