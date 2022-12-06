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
    RCLCPP_INFO(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    colorCamNode = pipeline->create<dai::node::ColorCamera>();
    paramHandler = std::make_unique<param_handlers::RGBParamHandler>(daiNodeName);
    paramHandler->declareParams(node, colorCamNode, socket, sensor);
    setXinXout(pipeline);
    RCLCPP_INFO(node->get_logger(), "Node %s created", daiNodeName.c_str());
};
void RGB::setNames() {
    colorQName = getName() + "_color";
    previewQName = getName() + "_preview";
    controlQName = getName() + "_control";
}

void RGB::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    if(paramHandler->get_param<bool>(getROSNode(), "i_publish_topic")) {
        xoutColor = pipeline->create<dai::node::XLinkOut>();
        xoutColor->setStreamName(colorQName);
        colorCamNode->video.link(xoutColor->input);
        if(paramHandler->get_param<bool>(getROSNode(), "i_enable_preview")) {
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
    if(paramHandler->get_param<bool>(getROSNode(), "i_publish_topic")) {
        colorQ = device->getOutputQueue(colorQName, paramHandler->get_param<int>(getROSNode(), "i_max_q_size"), false);
        colorQ->addCallback(std::bind(&RGB::colorQCB, this, std::placeholders::_1, std::placeholders::_2));
        rgbPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");
        if(paramHandler->get_param<bool>(getROSNode(), "i_enable_preview")) {
            previewQ = device->getOutputQueue(previewQName, paramHandler->get_param<int>(getROSNode(), "i_max_q_size"), false);
            previewQ->addCallback(std::bind(&RGB::colorQCB, this, std::placeholders::_1, std::placeholders::_2));
            previewPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/preview/image_raw");
        };
        auto calibHandler = device->readCalibration();
        rgbInfo = dai::ros::calibrationToCameraInfo(calibHandler,
                                                    static_cast<dai::CameraBoardSocket>(paramHandler->get_param<int>(getROSNode(), "i_board_socket_id")),
                                                    paramHandler->get_param<int>(getROSNode(), "i_width"),
                                                    paramHandler->get_param<int>(getROSNode(), "i_height"));
    }
    controlQ = device->getInputQueue(controlQName);
}

void RGB::closeQueues() {
    if(paramHandler->get_param<bool>(getROSNode(), "i_publish_topic")) {
        colorQ->close();
        if(paramHandler->get_param<bool>(getROSNode(), "i_enable_preview")) {
            previewQ->close();
        }
    }
    controlQ->close();
}

void RGB::colorQCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data) {
    auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    cv::Mat cv_frame = frame->getCvFrame();
    auto curr_time = getROSNode()->get_clock()->now();
    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    header.stamp = curr_time;
    header.frame_id = std::string(getROSNode()->get_name()) + "_rgb_camera_optical_frame";
    rgbInfo.header = header;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cv_frame);
    img_bridge.toImageMsg(img_msg);
    if(name == colorQName) {
        rgbPub.publish(img_msg, rgbInfo);
    } else {
        previewPub.publish(img_msg, rgbInfo);
    }
}

void RGB::link(const dai::Node::Input& in, int linkType) {
    if(linkType == static_cast<int>(link_types::RGBLinkType::color)) {
        colorCamNode->video.link(in);
    } else if(linkType == static_cast<int>(link_types::RGBLinkType::preview)) {
        colorCamNode->preview.link(in);
    }
}

dai::Node::Input RGB::getInput(int /*linkType*/) {
    throw(std::runtime_error("Class RGB has no input."));
}

void RGB::updateParams(const std::vector<rclcpp::Parameter>& params) {
    auto ctrl = paramHandler->setRuntimeParams(getROSNode(), params);
    controlQ->send(ctrl);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
