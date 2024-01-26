#include "depthai_ros_driver/dai_nodes/sensors/tof.hpp"

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/ToF.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/param_handlers/tof_param_handler.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
ToF::ToF(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    camNode = pipeline->create<dai::node::Camera>();
    tofNode = pipeline->create<dai::node::ToF>();
    ph = std::make_unique<param_handlers::ToFParamHandler>(node, daiNodeName);
    ph->declareParams(camNode, tofNode);
    setXinXout(pipeline);
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
}
ToF::~ToF() = default;
void ToF::setNames() {
    tofQName = getName() + "_tof";
}

void ToF::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    if(ph->getParam<bool>("i_publish_topic")) {
        xoutTof = pipeline->create<dai::node::XLinkOut>();
        xoutTof->setStreamName(tofQName);
        camNode->raw.link(tofNode->input);
        tofNode->depth.link(xoutTof->input);
    }
}

void ToF::setupQueues(std::shared_ptr<dai::Device> device) {
    if(ph->getParam<bool>("i_publish_topic")) {
        tofQ = device->getOutputQueue(tofQName, ph->getParam<int>("i_max_q_size"), false);
        auto tfPrefix = getTFPrefix(utils::getSocketName(static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"))));
        imageConverter =
            std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false, ph->getParam<bool>("i_get_base_device_timestamp"));
        imageConverter->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
        tofPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");
        infoManager = std::make_shared<camera_info_manager::CameraInfoManager>(
            getROSNode()->create_sub_node(std::string(getROSNode()->get_name()) + "/" + getName()).get(), "/" + getName());
        if(ph->getParam<std::string>("i_calibration_file").empty()) {
            infoManager->setCameraInfo(sensor_helpers::getCalibInfo(getROSNode()->get_logger(),
                                                                    *imageConverter,
                                                                    device,
                                                                    static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id")),
                                                                    ph->getParam<int>("i_width"),
                                                                    ph->getParam<int>("i_height")));
        } else {
            infoManager->loadCameraInfo(ph->getParam<std::string>("i_calibration_file"));
        }
        tofQ->addCallback(std::bind(sensor_helpers::cameraPub,
                                    std::placeholders::_1,
                                    std::placeholders::_2,
                                    *imageConverter,
                                    tofPub,
                                    infoManager,
                                    ph->getParam<bool>("i_enable_lazy_publisher")));
    }
}
void ToF::closeQueues() {
    if(ph->getParam<bool>("i_publish_topic")) {
        tofQ->close();
    }
}

void ToF::link(dai::Node::Input in, int /*linkType*/) {
    tofNode->depth.link(in);
}

void ToF::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
