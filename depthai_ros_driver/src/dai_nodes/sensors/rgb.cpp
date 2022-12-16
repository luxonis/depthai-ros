#include "depthai_ros_driver/dai_nodes/sensors/rgb.hpp"

#include "cv_bridge/cv_bridge.h"
#include "depthai_bridge/ImageConverter.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
RGB::RGB(const std::string& daiNodeName,
         rclcpp::Node* node,
         std::shared_ptr<dai::Pipeline> pipeline,
         dai::CameraBoardSocket socket = dai::CameraBoardSocket::RGB,
         sensor_helpers::ImageSensor sensor = {"IMX378", {"12mp", "4k"}, true})
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    colorCamNode = pipeline->create<dai::node::ColorCamera>();
    ph = std::make_unique<param_handlers::RGBParamHandler>(daiNodeName);
    ph->declareParams(node, colorCamNode, socket, sensor);
    setXinXout(pipeline);
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
};
void RGB::setNames() {
    ispQName = getName() + "_isp";
    previewQName = getName() + "_preview";
    controlQName = getName() + "_control";
}

void RGB::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    if(ph->getParam<bool>(getROSNode(), "i_publish_topic")) {
        xoutColor = pipeline->create<dai::node::XLinkOut>();
        xoutColor->setStreamName(ispQName);
        colorCamNode->isp.link(xoutColor->input);
        if(ph->getParam<bool>(getROSNode(), "i_enable_preview")) {
            xoutPreview = pipeline->create<dai::node::XLinkOut>();
            xoutPreview->setStreamName(previewQName);
            xoutPreview->input.setQueueSize(2);
            xoutPreview->input.setBlocking(false);
            colorCamNode->preview.link(xoutPreview->input);
        }
    }
    xinControl = pipeline->create<dai::node::XLinkIn>();
    xinControl->setStreamName(controlQName);
    xinControl->out.link(colorCamNode->inputControl);
}

void RGB::setupQueues(std::shared_ptr<dai::Device> device) {
    auto calibHandler = device->readCalibration();
    if(ph->getParam<bool>(getROSNode(), "i_publish_topic")) {
        auto tfPrefix = std::string(getROSNode()->get_name()) + "_" + getName();
        imageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
        colorQ = device->getOutputQueue(ispQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
        colorQ->addCallback(std::bind(&RGB::colorQCB, this, std::placeholders::_1, std::placeholders::_2));
        rgbPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");

        if(ph->getParam<bool>(getROSNode(), "i_enable_preview")) {
            previewQ = device->getOutputQueue(previewQName, ph->getParam<int>(getROSNode(), "i_max_q_size"), false);
            previewQ->addCallback(std::bind(&RGB::colorQCB, this, std::placeholders::_1, std::placeholders::_2));
            previewPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/preview/image_raw");
            previewInfo = dai::ros::calibrationToCameraInfo(calibHandler,
                                                            static_cast<dai::CameraBoardSocket>(ph->getParam<int>(getROSNode(), "i_board_socket_id")),
                                                            ph->getParam<int>(getROSNode(), "i_preview_size"),
                                                            ph->getParam<int>(getROSNode(), "i_preview_size"));
        };
        rgbInfo = dai::ros::calibrationToCameraInfo(calibHandler,
                                                    static_cast<dai::CameraBoardSocket>(ph->getParam<int>(getROSNode(), "i_board_socket_id")),
                                                    ph->getParam<int>(getROSNode(), "i_width"),
                                                    ph->getParam<int>(getROSNode(), "i_height"));
    }
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

void RGB::colorQCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data) {
    auto img = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    std::deque<sensor_msgs::msg::Image> deq;
    imageConverter->toRosMsg(img, deq);
    while(deq.size() > 0) {
        auto currMsg = deq.front();
        // currMsg.header.stamp = getROSNode()->get_clock()->now();
        if(name == ispQName) {
            rgbInfo.header = currMsg.header;
            rgbPub.publish(currMsg, rgbInfo);
        } else {
            previewInfo.header = currMsg.header;
            previewPub.publish(currMsg, previewInfo);
        }
        deq.pop_front();
    }
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

dai::Node::Input RGB::getInput(int /*linkType*/) {
    throw(std::runtime_error("Class RGB has no input."));
}

void RGB::updateParams(const std::vector<rclcpp::Parameter>& params) {
    auto ctrl = ph->setRuntimeParams(getROSNode(), params);
    controlQ->send(ctrl);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
