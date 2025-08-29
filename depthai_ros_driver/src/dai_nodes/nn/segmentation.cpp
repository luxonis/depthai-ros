#include "depthai_ros_driver/dai_nodes/nn/segmentation.hpp"

#include <cstdint>
#include <depthai/common/CameraBoardSocket.hpp>
#include <depthai/modelzoo/Zoo.hpp>
#include <depthai/nn_archive/NNArchive.hpp>

#include "camera_info_manager/camera_info_manager.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn {

Segmentation::Segmentation(const std::string& daiNodeName,
                           std::shared_ptr<rclcpp::Node> node,
                           std::shared_ptr<dai::Pipeline> pipeline,
                           const std::string& deviceName,
                           bool rsCompat,
                           dai_nodes::SensorWrapper& camNode,
                           const dai::CameraBoardSocket& socket)
    : BaseNode(daiNodeName, node, pipeline, deviceName, rsCompat) {
    RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    ph = std::make_unique<param_handlers::NNParamHandler>(node, daiNodeName, deviceName, rsCompat, socket);
    ph->declareParams(segNode);
    description = std::make_shared<dai::NNModelDescription>();
    description->model = ph->getParam<std::string>("i_nn_model");
    segNode = pipeline->create<dai::node::NeuralNetwork>()->build(camNode.getUnderlyingNode(), *description);
    imageManip = pipeline->create<dai::node::ImageManip>();
    RCLCPP_DEBUG(getLogger(), "Node %s created", daiNodeName.c_str());
    imageManip->out.link(segNode->input);
    setInOut(pipeline);
}

Segmentation::~Segmentation() = default;

void Segmentation::setNames() {
    nnQName = getName() + "_nn";
    ptQName = getName() + "_pt";
}

void Segmentation::setInOut(std::shared_ptr<dai::Pipeline> pipeline) {}

void Segmentation::setupQueues(std::shared_ptr<dai::Device> device) {
    nnQ = segNode->out.createOutputQueue(ph->getParam<int>("i_max_q_size"), false);
    nnPub = image_transport::create_camera_publisher(getROSNode().get(), "~/" + getName() + "/image_raw");
    nnQ->addCallback(std::bind(&Segmentation::segmentationCB, this, std::placeholders::_1, std::placeholders::_2));
    if(ph->getParam<bool>("i_enable_passthrough")) {
        auto tfPrefix = getOpticalFrameName(getSocketName(ph->getSocketID()));
        ptQ = segNode->passthrough.createOutputQueue(ph->getParam<int>("i_max_q_size"), false);
        imageConverter = std::make_unique<depthai_bridge::ImageConverter>(tfPrefix, false);
        infoManager = std::make_shared<camera_info_manager::CameraInfoManager>(
            getROSNode()->create_sub_node(std::string(getROSNode()->get_name()) + "/" + getName()).get(), "/" + getName());
        infoManager->setCameraInfo(sensor_helpers::getCalibInfo(getROSNode()->get_logger(), imageConverter, device->readCalibration(), ph->getSocketID()));

        ptPub = image_transport::create_camera_publisher(getROSNode().get(), "~/" + getName() + "/passthrough/image_raw");
        ptQ->addCallback(std::bind(sensor_helpers::basicCameraPub, std::placeholders::_1, std::placeholders::_2, *imageConverter, ptPub, infoManager));
    }
}

void Segmentation::closeQueues() {
    nnQ->close();
    if(ph->getParam<bool>("i_enable_passthrough")) {
        ptQ->close();
    }
}
cv::Mat xarray_to_mat(xt::xarray<int> xarr) {
    cv::Mat mat(xarr.shape()[0], xarr.shape()[1], CV_32SC1, xarr.data());
    return mat;
}
void Segmentation::segmentationCB(const std::string& name, const std::shared_ptr<dai::ADatatype>& data) {
    auto seg = std::dynamic_pointer_cast<dai::NNData>(data);
    auto layers = seg->getAllLayerNames();
    auto outputName = layers[0];
    auto nnFrame = seg->getTensor<int32_t>(outputName, true);
    auto [width, height] = seg->transformation->getSize();
    nnFrame.reshape({width, height});
    cv::Mat nn_mat = cv::Mat(nnFrame.shape()[0], nnFrame.shape()[1], CV_32SC1, nnFrame.data());
    auto classNum = seg->getTensor<int32_t>(layers[1], true).shape()[1];
    cv::Mat cv_frame = decodeDeeplab(nn_mat, classNum);
    auto currTime = getROSNode()->get_clock()->now();
    cv_bridge::CvImage imgBridge;
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;
    header.stamp = getROSNode()->get_clock()->now();
    auto tfPrefix = getOpticalFrameName(getSocketName(static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"))));
    header.frame_id = tfPrefix;
    nnInfo.header = header;
    imgBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cv_frame);
    imgBridge.toImageMsg(img_msg);
    nnPub.publish(img_msg, nnInfo);
}
cv::Mat Segmentation::decodeDeeplab(cv::Mat mat, int classNum) {
    cv::Mat out = mat.mul(255 / classNum);
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
void Segmentation::link(dai::Node::Input& in, int /*linkType*/) {
    segNode->out.link(in);
}

dai::Node::Input& Segmentation::getInput(int /*linkType*/) {
    if(ph->getParam<bool>("i_disable_resize")) {
        return segNode->input;
    }
    return imageManip->inputImage;
}

}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
