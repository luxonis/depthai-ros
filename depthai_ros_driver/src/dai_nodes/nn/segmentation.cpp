#include "depthai_ros_driver/dai_nodes/nn/segmentation.hpp"

#include "camera_info_manager/camera_info_manager.hpp"
#include "cv_bridge/cv_bridge.h"
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn {

Segmentation::Segmentation(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    segNode = pipeline->create<dai::node::NeuralNetwork>();
    imageManip = pipeline->create<dai::node::ImageManip>();
    ph = std::make_unique<param_handlers::NNParamHandler>(node, daiNodeName);
    ph->declareParams(segNode, imageManip);
    RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
    imageManip->out.link(segNode->input);
    setXinXout(pipeline);
}

Segmentation::~Segmentation() = default;

void Segmentation::setNames() {
    nnQName = getName() + "_nn";
    ptQName = getName() + "_pt";
}

void Segmentation::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    xoutNN = pipeline->create<dai::node::XLinkOut>();
    xoutNN->setStreamName(nnQName);
    segNode->out.link(xoutNN->input);
    if(ph->getParam<bool>("i_enable_passthrough")) {
        xoutPT = pipeline->create<dai::node::XLinkOut>();
        xoutPT->setStreamName(ptQName);
        segNode->passthrough.link(xoutPT->input);
    }
}

void Segmentation::setupQueues(std::shared_ptr<dai::Device> device) {
    nnQ = device->getOutputQueue(nnQName, ph->getParam<int>("i_max_q_size"), false);
    nnPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/image_raw");
    nnQ->addCallback(std::bind(&Segmentation::segmentationCB, this, std::placeholders::_1, std::placeholders::_2));
    if(ph->getParam<bool>("i_enable_passthrough")) {
        auto tfPrefix = getTFPrefix("rgb");
        ptQ = device->getOutputQueue(ptQName, ph->getParam<int>("i_max_q_size"), false);
        imageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
        infoManager = std::make_shared<camera_info_manager::CameraInfoManager>(
            getROSNode()->create_sub_node(std::string(getROSNode()->get_name()) + "/" + getName()).get(), "/" + getName());
        infoManager->setCameraInfo(sensor_helpers::getCalibInfo(getROSNode()->get_logger(),
                                                                *imageConverter,
                                                                device,
                                                                dai::CameraBoardSocket::RGB,
                                                                imageManip->initialConfig.getResizeWidth(),
                                                                imageManip->initialConfig.getResizeWidth()));

        ptPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/passthrough/image_raw");
        ptQ->addCallback(std::bind(sensor_helpers::imgCB, std::placeholders::_1, std::placeholders::_2, *imageConverter, ptPub, infoManager));
    }
}

void Segmentation::closeQueues() {
    nnQ->close();
    if(ph->getParam<bool>("i_enable_passthrough")) {
        ptQ->close();
    }
}

void Segmentation::segmentationCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
    auto in_det = std::dynamic_pointer_cast<dai::NNData>(data);
    std::vector<std::int32_t> nn_frame = in_det->getFirstLayerInt32();
    cv::Mat nn_mat = cv::Mat(nn_frame);
    nn_mat = nn_mat.reshape(0, 256);
    cv::Mat cv_frame = decodeDeeplab(nn_mat);
    auto currTime = getROSNode()->get_clock()->now();
    cv_bridge::CvImage imgBridge;
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    header.stamp = getROSNode()->get_clock()->now();
    header.frame_id = std::string(getROSNode()->get_name()) + "_rgb_camera_optical_frame";
    nnInfo.header = header;
    imgBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cv_frame);
    imgBridge.toImageMsg(img_msg);
    nnPub.publish(img_msg, nnInfo);
}
cv::Mat Segmentation::decodeDeeplab(cv::Mat mat) {
    cv::Mat out = mat.mul(255 / 21);
    out.convertTo(out, CV_8UC1);
    cv::Mat colors = cv::Mat(256, 1, CV_8UC3);
    cv::applyColorMap(out, colors, cv::COLORMAP_JET);
    for(int row = 0; row < out.rows; ++row) {
        uchar* p = out.ptr(row);
        for(int col = 0; col < out.cols; ++col) {
            if(*p++ == 0) {
                colors.at<cv::Vec3b>(row, col)[0] = 0;
                colors.at<cv::Vec3b>(row, col)[1] = 0;
                colors.at<cv::Vec3b>(row, col)[2] = 0;
            }
        }
    }
    return colors;
}
void Segmentation::link(dai::Node::Input in, int /*linkType*/) {
    segNode->out.link(in);
}

dai::Node::Input Segmentation::getInput(int /*linkType*/) {
    if(ph->getParam<bool>("i_disable_resize")) {
        return segNode->input;
    }
    return imageManip->inputImage;
}

void Segmentation::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(params);
}
}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver