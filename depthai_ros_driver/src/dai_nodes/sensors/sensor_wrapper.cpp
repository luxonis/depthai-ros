#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai_bridge/ImageConverter.hpp"
// #include "depthai_ros_driver/dai_nodes/nn/nn_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/camera.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/feature_tracker.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/thermal.hpp"
#include "depthai_ros_driver/param_handlers/sensor_param_handler.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
SensorWrapper::SensorWrapper(const std::string& daiNodeName,
                             std::shared_ptr<rclcpp::Node> node,
                             std::shared_ptr<dai::Pipeline> pipeline,
                             const std::string& deviceName,
                             bool rsCompat,
                             dai::CameraBoardSocket socket,
                             bool publish)
    : BaseNode(daiNodeName, node, pipeline, deviceName, rsCompat) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s base", daiNodeName.c_str());
    ph = std::make_unique<param_handlers::SensorParamHandler>(node, daiNodeName, deviceName, rsCompat, socket);

    if(ph->getParam<bool>("i_simulate_from_topic")) {
        std::string topicName = ph->getParam<std::string>("i_simulated_topic_name");
        if(topicName.empty()) {
            topicName = "~/" + getName() + "/input";
        }
        sub = node->create_subscription<sensor_msgs::msg::Image>(topicName, 10, std::bind(&SensorWrapper::subCB, this, std::placeholders::_1));
        converter = std::make_unique<depthai_bridge::ImageConverter>("sub", true);
        setNames();
        setInOut(pipeline);
    }

    socketID = ph->getParam<int>("i_board_socket_id");
    if(ph->getParam<bool>("i_disable_node") && ph->getParam<bool>("i_simulate_from_topic")) {
        RCLCPP_INFO(getROSNode()->get_logger(), "Disabling node %s, pipeline data taken from topic.", getName().c_str());
    } else {
        if(ph->getParam<bool>("i_disable_node")) {
            RCLCPP_WARN(getROSNode()->get_logger(), "For node to be disabled, %s.i_simulate_from_topic must be set to true.", getName().c_str());
        }
        sensorNode = std::make_unique<Camera>(daiNodeName, node, pipeline, deviceName, rsCompat, socket, publish);
    }
    if(ph->getParam<bool>("i_enable_feature_tracker")) {
        featureTrackerNode = std::make_unique<FeatureTracker>(daiNodeName + std::string("_feature_tracker"), node, pipeline, deviceName, rsCompat);
        sensorNode->link(featureTrackerNode->getInput());
    }
    if(ph->getParam<bool>("i_enable_nn")) {
        // nnNode = std::make_unique<NNWrapper>(daiNodeName + std::string("_nn"), node, pipeline, static_cast<dai::CameraBoardSocket>(socketID));
        // sensorNode->link(nnNode->getInput(), static_cast<int>(link_types::RGBLinkType::preview));
    }
    RCLCPP_DEBUG(node->get_logger(), "Base node %s created", daiNodeName.c_str());
}
SensorWrapper::~SensorWrapper() = default;

void SensorWrapper::subCB(const sensor_msgs::msg::Image& img) {
    dai::ImgFrame data;
    converter->toDaiMsg(img, data);
    data.setInstanceNum(socketID);
    // inQ->send(data);
}
void SensorWrapper::setNames() {
    inQName = getName() + "_topic_in";
}

void SensorWrapper::setInOut(std::shared_ptr<dai::Pipeline> pipeline) {
    // xIn = pipeline->create<dai::node::XLinkIn>();
    // xIn->setStreamName(inQName);
}

void SensorWrapper::setupQueues(std::shared_ptr<dai::Device> device) {
    if(ph->getParam<bool>("i_simulate_from_topic")) {
        // inQ = device->getInputQueue(inQName, ph->getParam<int>("i_max_q_size"), false);
    }
    if(!ph->getParam<bool>("i_disable_node")) {
        sensorNode->setupQueues(device);
    }
    if(ph->getParam<bool>("i_enable_feature_tracker")) {
        featureTrackerNode->setupQueues(device);
    }
    if(ph->getParam<bool>("i_enable_nn")) {
        nnNode->setupQueues(device);
    }
}
void SensorWrapper::closeQueues() {
    if(ph->getParam<bool>("i_simulate_from_topic")) {
        inQ->close();
    }
    if(!ph->getParam<bool>("i_disable_node")) {
        sensorNode->closeQueues();
    }
    if(ph->getParam<bool>("i_enable_feature_tracker")) {
        featureTrackerNode->closeQueues();
    }
    if(ph->getParam<bool>("i_enable_nn")) {
        nnNode->closeQueues();
    }
}

void SensorWrapper::link(dai::Node::Input in, int linkType) {
    if(ph->getParam<bool>("i_simulate_from_topic")) {
        // xIn->out.link(in);
    } else {
        sensorNode->link(in, linkType);
    }
}

std::shared_ptr<dai::node::Camera> SensorWrapper::getUnderlyingNode() {
    return sensorNode->getUnderlyingNode();
}

dai::Node::Output* SensorWrapper::getDefaultOut() {
    return sensorNode->getDefaultOut();
}

std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> SensorWrapper::getPublishers() {
    if(ph->getParam<bool>("i_disable_node")) {
        return std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>>();
    }
    return sensorNode->getPublishers();
}

void SensorWrapper::updateParams(const std::vector<rclcpp::Parameter>& params) {
    sensorNode->updateParams(params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
