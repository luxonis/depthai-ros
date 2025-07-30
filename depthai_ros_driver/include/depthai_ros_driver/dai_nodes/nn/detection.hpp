#pragma once

#include <depthai/modelzoo/Zoo.hpp>
#include <depthai/nn_archive/NNArchive.hpp>
#include <memory>
#include <string>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {

namespace dai_nodes {
namespace nn {
class Detection : public BaseNode {
   public:
    /**
     * @brief      Constructor of the class Detection. Creates a DetectionNetwork node in the pipeline. Also creates an ImageManip node in the pipeline.
     *             The ImageManip node is used to resize the input frames to the size required by the DetectionNetwork node.
     *
     * @param[in]  daiNodeName  The dai node name
     * @param      node         The node
     * @param      pipeline     The pipeline
     */
    Detection(const std::string& daiNodeName,
              std::shared_ptr<rclcpp::Node> node,
              std::shared_ptr<dai::Pipeline> pipeline,
              const std::string& deviceName,
              bool rsCompat,
              dai_nodes::SensorWrapper& camNode,
              const dai::CameraBoardSocket& socket = dai::CameraBoardSocket::CAM_A)
        : BaseNode(daiNodeName, node, pipeline, deviceName, rsCompat) {
        RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
        setNames();
        detectionNode = pipeline->create<dai::node::DetectionNetwork>();
        ph = std::make_unique<param_handlers::NNParamHandler>(node, daiNodeName, deviceName, rsCompat, socket);
        ph->declareParams(detectionNode);
        dai::NNModelDescription description;
        description.model = ph->getParam<std::string>("i_nn_model");
        detectionNode->build(camNode.getUnderlyingNode(), description);

        RCLCPP_DEBUG(getLogger(), "Node %s created", daiNodeName.c_str());
        setInOut(pipeline);
    }
    ~Detection() = default;
    /**
     * @brief      Sets up the queues for the DetectionNetwork node and the ImageManip node. Also sets up the publishers for the DetectionNetwork node and the
     * ImageManip node.
     *
     * @param      device  The device
     */
    void setupQueues(std::shared_ptr<dai::Device> device) override {
        nnQ = detectionNode->out.createOutputQueue(ph->getParam<int>("i_max_q_size"), false);
        std::string socketName = getSocketName(static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id")));
        auto tfPrefix = getOpticalFrameName(socketName);

        detConverter = std::make_unique<depthai_bridge::ImgDetectionConverter>(tfPrefix, false, ph->getParam<bool>("i_get_base_device_timestamp"));
        detConverter->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
        rclcpp::PublisherOptions options;
        options.qos_overriding_options = rclcpp::QosOverridingOptions();
        detPub = getROSNode()->template create_publisher<vision_msgs::msg::Detection2DArray>("~/" + getName() + "/detections", 10, options);
        nnQ->addCallback(std::bind(&Detection::detectionCB, this, std::placeholders::_1, std::placeholders::_2));

        if(ph->getParam<bool>("i_enable_passthrough")) {
            utils::ImgConverterConfig convConf;
            convConf.getBaseDeviceTimestamp = ph->getParam<bool>("i_get_base_device_timestamp");
            convConf.tfPrefix = tfPrefix;
            convConf.updateROSBaseTimeOnRosMsg = ph->getParam<bool>("i_update_ros_base_time_on_ros_msg");

            utils::ImgPublisherConfig pubConf;
            pubConf.daiNodeName = getName();
            pubConf.topicName = "~/" + getName() + "/passthrough";
            pubConf.infoSuffix = "/passthrough";
            pubConf.socket = static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"));

            ptPub->setup(device, convConf, pubConf);
        }
    };
    /**
     * @brief      Links the input of the DetectionNetwork node to the output of the ImageManip node.
     *
     * @param[in]  in        The input of the DetectionNetwork node
     * @param[in]  linkType  The link type (not used)
     */
    void link(dai::Node::Input in, int /*linkType*/) override {
        detectionNode->out.link(in);
    };
    /**
     * @brief      Gets the input of the DetectionNetwork node.
     *
     * @param[in]  linkType  The link type (not used)
     *
     * @return     The input of the DetectionNetwork node.
     */
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
    /**
     * @brief      Sets the XLinkOut for the DetectionNetwork node and the ImageManip node. Additionally enables the passthrough.
     *
     * @param      pipeline  The pipeline
     */
    void setInOut(std::shared_ptr<dai::Pipeline> pipeline) override {
        if(ph->getParam<bool>("i_enable_passthrough")) {
            ptPub = setupOutput(pipeline, ptQName, &detectionNode->passthrough);
        }
    };
    /**
     * @brief      Closes the queues for the DetectionNetwork node and the passthrough.
     */
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
    /**
     * @brief      Callback for the DetectionNetwork node. Converts the ImgDetections to Detection2DArray and publishes it.
     *
     * @param[in]  name  The name of the stream
     * @param[in]  data  The DAI data
     */
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
    std::unique_ptr<depthai_bridge::ImgDetectionConverter> detConverter;
    std::vector<std::string> labelNames;
    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detPub;
    std::shared_ptr<sensor_helpers::ImagePublisher> ptPub;
    std::shared_ptr<dai::node::DetectionNetwork> detectionNode;
    std::shared_ptr<dai::node::ImageManip> imageManip;
    std::unique_ptr<param_handlers::NNParamHandler> ph;
    std::shared_ptr<dai::MessageQueue> nnQ, ptQ;
    std::string nnQName, ptQName;
};

}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
