#pragma once

#include <memory>
#include <string>
#include <vector>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/nn/nn_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "image_transport/camera_publisher.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn {
template <typename T>
class SpatialDetection : public BaseNode {
   public:
    SpatialDetection(const std::string& daiNodeName, rclcpp::Node* node, std::shared_ptr<dai::Pipeline> pipeline) : BaseNode(daiNodeName, node, pipeline) {
        RCLCPP_DEBUG(node->get_logger(), "Creating node %s", daiNodeName.c_str());
        setNames();
        spatialNode = pipeline->create<T>();
        imageManip = pipeline->create<dai::node::ImageManip>();
        ph = std::make_unique<param_handlers::NNParamHandler>(node, daiNodeName);
        ph->declareParams(spatialNode, imageManip);
        RCLCPP_DEBUG(node->get_logger(), "Node %s created", daiNodeName.c_str());
        imageManip->out.link(spatialNode->input);
        setXinXout(pipeline);
    }
    ~SpatialDetection() = default;
    void updateParams(const std::vector<rclcpp::Parameter>& params) override {
        ph->setRuntimeParams(params);
    };
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
        detConverter = std::make_unique<dai::ros::SpatialDetectionConverter>(
            tfPrefix + "_camera_optical_frame", width, height, false, ph->getParam<bool>("i_get_base_device_timestamp"));
        detConverter->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
        nnQ->addCallback(std::bind(&SpatialDetection::spatialCB, this, std::placeholders::_1, std::placeholders::_2));
        rclcpp::PublisherOptions options;
        options.qos_overriding_options = rclcpp::QosOverridingOptions();
        detPub = getROSNode()->template create_publisher<vision_msgs::msg::Detection3DArray>("~/" + getName() + "/spatial_detections", 10, options);

        if(ph->getParam<bool>("i_enable_passthrough")) {
            ptQ = device->getOutputQueue(ptQName, ph->getParam<int>("i_max_q_size"), false);
            ptImageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
            ptImageConverter->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
            ptInfoMan = std::make_shared<camera_info_manager::CameraInfoManager>(
                getROSNode()->create_sub_node(std::string(getROSNode()->get_name()) + "/" + getName()).get(), "/" + getName());
            ptInfoMan->setCameraInfo(sensor_helpers::getCalibInfo(getROSNode()->get_logger(),
                                                                  *ptImageConverter,
                                                                  device,
                                                                  static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id")),
                                                                  width,
                                                                  height));

            ptPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/passthrough/image_raw");
            ptQ->addCallback(std::bind(sensor_helpers::imgCB, std::placeholders::_1, std::placeholders::_2, *ptImageConverter, ptPub, ptInfoMan));
        }

        if(ph->getParam<bool>("i_enable_passthrough_depth")) {
            dai::CameraBoardSocket socket = static_cast<dai::CameraBoardSocket>(getROSNode()->get_parameter("stereo.i_board_socket_id").as_int());
            if(!getROSNode()->get_parameter("stereo.i_align_depth").as_bool()) {
                tfPrefix = getTFPrefix("right");
            };
            ptDepthQ = device->getOutputQueue(ptDepthQName, ph->getParam<int>("i_max_q_size"), false);
            ptDepthImageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
            ptDepthImageConverter->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
            ptDepthInfoMan = std::make_shared<camera_info_manager::CameraInfoManager>(
                getROSNode()->create_sub_node(std::string(getROSNode()->get_name()) + "/" + getName()).get(), "/" + getName());
            ptDepthInfoMan->setCameraInfo(sensor_helpers::getCalibInfo(getROSNode()->get_logger(),
                                                                       *ptDepthImageConverter,
                                                                       device,
                                                                       socket,
                                                                       getROSNode()->get_parameter("stereo.i_width").as_int(),
                                                                       getROSNode()->get_parameter("stereo.i_height").as_int()));

            ptDepthPub = image_transport::create_camera_publisher(getROSNode(), "~/" + getName() + "/passthrough_depth/image_raw");
            ptDepthQ->addCallback(
                std::bind(sensor_helpers::imgCB, std::placeholders::_1, std::placeholders::_2, *ptDepthImageConverter, ptDepthPub, ptDepthInfoMan));
        }
    };
    void link(dai::Node::Input in, int /*linkType = 0*/) override {
        spatialNode->out.link(in);
    };
    dai::Node::Input getInput(int linkType = 0) override {
        if(linkType == static_cast<int>(nn_helpers::link_types::SpatialNNLinkType::input)) {
            if(ph->getParam<bool>("i_disable_resize")) {
                return spatialNode->input;
            }
            return imageManip->inputImage;
        } else {
            return spatialNode->inputDepth;
        }
    };
    void setNames() override {
        nnQName = getName() + "_nn";
        ptQName = getName() + "_pt";
        ptDepthQName = getName() + "_pt_depth";
    };
    void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) override {
        xoutNN = pipeline->create<dai::node::XLinkOut>();
        xoutNN->setStreamName(nnQName);
        spatialNode->out.link(xoutNN->input);
        if(ph->getParam<bool>("i_enable_passthrough")) {
            xoutPT = pipeline->create<dai::node::XLinkOut>();
            xoutPT->setStreamName(ptQName);
            spatialNode->passthrough.link(xoutPT->input);
        }
        if(ph->getParam<bool>("i_enable_passthrough_depth")) {
            xoutPTDepth = pipeline->create<dai::node::XLinkOut>();
            xoutPTDepth->setStreamName(ptDepthQName);
            spatialNode->passthroughDepth.link(xoutPTDepth->input);
        }
    };
    void closeQueues() override {
        nnQ->close();
        if(ph->getParam<bool>("i_enable_passthrough")) {
            ptQ->close();
        }
        if(ph->getParam<bool>("i_enable_passthrough_depth")) {
            ptDepthQ->close();
        }
    };

   private:
    void spatialCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
        auto inDet = std::dynamic_pointer_cast<dai::SpatialImgDetections>(data);
        std::deque<vision_msgs::msg::Detection3DArray> deq;
        detConverter->toRosVisionMsg(inDet, deq);
        while(deq.size() > 0) {
            auto currMsg = deq.front();
            detPub->publish(currMsg);
            deq.pop_front();
        }
    };
    std::unique_ptr<dai::ros::SpatialDetectionConverter> detConverter;
    std::vector<std::string> labelNames;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detPub;
    std::unique_ptr<dai::ros::ImageConverter> ptImageConverter, ptDepthImageConverter;
    image_transport::CameraPublisher ptPub, ptDepthPub;
    sensor_msgs::msg::CameraInfo ptInfo, ptDepthInfo;
    std::shared_ptr<camera_info_manager::CameraInfoManager> ptInfoMan, ptDepthInfoMan;
    std::shared_ptr<T> spatialNode;
    std::shared_ptr<dai::node::ImageManip> imageManip;
    std::unique_ptr<param_handlers::NNParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> nnQ, ptQ, ptDepthQ;
    std::shared_ptr<dai::node::XLinkOut> xoutNN, xoutPT, xoutPTDepth;
    std::string nnQName, ptQName, ptDepthQName;
};

}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver