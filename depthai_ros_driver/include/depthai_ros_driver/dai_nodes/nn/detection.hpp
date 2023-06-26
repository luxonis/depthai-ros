#pragma once

#include <memory>
#include <string>
#include <vector>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {

namespace dai_nodes {
namespace nn {
template <typename T>
class Detection : public BaseNode {
   public:
    Detection(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
        RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
        setNames();
        detectionNode = pipeline->create<T>();
        imageManip = pipeline->create<dai::node::ImageManip>();
        ph = std::make_unique<param_handlers::NNParamHandler>(node, daiNodeName);
        ph->declareParams(detectionNode, imageManip);
        RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
        imageManip->out.link(detectionNode->input);
        setXinXout(pipeline);
    }
    ~Detection() = default;

    void setupQueues(std::shared_ptr<dai::Device> device) override {
        nnQ = device->getOutputQueue(nnQName, ph->getParam<int>("i_max_q_size"), false);
        auto tfPrefix = getTFPrefix("rgb");
        int width;
        int height;
        if(ph->getParam<bool>("i_disable_resize")) {
            width = getROSNode()->get_parameter("rgb.i_preview_size").as_int();
            height = getROSNode()->get_parameter("rgb.i_preview_size").as_int();
        } else {
            width = imageManip->initialConfig.getResizeConfig().width;
            height = imageManip->initialConfig.getResizeConfig().height;
        }
        detConverter = std::make_unique<dai::ros::ImgDetectionConverter>(
            tfPrefix + "_camera_optical_frame", width, height, false, ph->getParam<bool>("i_get_base_device_timestamp"));
        detConverter->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
        rclcpp::PublisherOptions options;
        options.qos_overriding_options = rclcpp::QosOverridingOptions();
        detPub = getROSNode()->template create_publisher<vision_msgs::msg::Detection2DArray>("~/" + getName() + "/detections", 10, options);
        nnQ->addCallback(std::bind(&Detection::detectionCB, this, std::placeholders::_1, std::placeholders::_2));

        if(ph->getParam<bool>("i_enable_passthrough")) {
            ptQ = device->getOutputQueue(ptQName, ph->getParam<int>("i_max_q_size"), false);
            imageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
            imageConverter->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
            infoManager = std::make_shared<camera_info_manager::CameraInfoManager>(
                getROSNode()->create_sub_node(std::string(getROSNode()->get_name()) + "/" + getName()).get(), "/" + getName());
            infoManager->setCameraInfo(sensor_helpers::getCalibInfo(getROSNode()->get_logger(),
                                                                    *imageConverter,
                                                                    device,
                                                                    static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id")),
                                                                    width,
                                                                    height));

            ptPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/passthrough/image_raw");
            ptQ->addCallback(std::bind(sensor_helpers::imgCB, std::placeholders::_1, std::placeholders::_2, *imageConverter, ptPub, infoManager));
        }
    };
    void link(dai::Node::Input in, int /*linkType*/) override {
        detectionNode->out.link(in);
    };
    dai::Node::Input getInput(int /*linkType*/) override {
        if(ph->getParam<bool>("i_disable_resize")) {
            return detectionNode->input;
        }
        return imageManip->inputImage;
    };
    void setNames() override {
        nnQName = getName() + "_nn";
        ptQName = getName() + "_pt";
    };
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override {
        xoutNN = pipeline->create<dai::node::XLinkOut>();
        xoutNN->setStreamName(nnQName);
        detectionNode->out.link(xoutNN->input);
        if(ph->getParam<bool>("i_enable_passthrough")) {
            xoutPT = pipeline->create<dai::node::XLinkOut>();
            xoutPT->setStreamName(ptQName);
            detectionNode->passthrough.link(xoutPT->input);
        }
    };
    void closeQueues() override {
        nnQ->close();
        if(ph->getParam<bool>("i_enable_passthrough")) {
            ptQ->close();
        }
    };

    void updateParams(const std::vector<rclcpp::Parameter>& params) override {
        ph->setRuntimeParams(params);
    };

   private:
    void detectionCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
        auto inDet = std::dynamic_pointer_cast<dai::ImgDetections>(data);
        std::deque<vision_msgs::msg::Detection2DArray> deq;
        detConverter->toRosMsg(inDet, deq);
        while(deq.size() > 0) {
            auto currMsg = deq.front();
            detPub->publish(currMsg);
            deq.pop_front();
        }
    };
    std::unique_ptr<dai::ros::ImgDetectionConverter> detConverter;
    std::vector<std::string> labelNames;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detPub;
    std::unique_ptr<dai::ros::ImageConverter> imageConverter;
    image_transport::CameraPublisher ptPub;
    std::shared_ptr<camera_info_manager::CameraInfoManager> infoManager;
    std::shared_ptr<T> detectionNode;
    std::shared_ptr<dai::node::ImageManip> imageManip;
    std::unique_ptr<param_handlers::NNParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> nnQ, ptQ;
    std::shared_ptr<dai::node::XLinkOut> xoutNN, xoutPT;
    std::string nnQName, ptQName;
};

}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver