#pragma once

#include <memory>
#include <string>
#include <vector>

#include "camera_info_manager/camera_info_manager.h"
#include "depthai-shared/common/CameraBoardSocket.hpp"
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
#include "depthai_ros_driver/parametersConfig.h"
#include "depthai_ros_driver/utils.hpp"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"
#include "ros/node_handle.h"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn {
template <typename T>
class SpatialDetection : public BaseNode {
   public:
    SpatialDetection(const std::string& daiNodeName,
                     ros::NodeHandle node,
                     std::shared_ptr<dai::Pipeline> pipeline,
                     const dai::CameraBoardSocket& socket = dai::CameraBoardSocket::CAM_A)
        : BaseNode(daiNodeName, node, pipeline), it(node) {
        ROS_DEBUG("Creating node %s", daiNodeName.c_str());
        setNames();
        spatialNode = pipeline->create<T>();
        imageManip = pipeline->create<dai::node::ImageManip>();
        ph = std::make_unique<param_handlers::NNParamHandler>(node, daiNodeName, socket);
        ph->declareParams(spatialNode, imageManip);
        ROS_DEBUG("Node %s created", daiNodeName.c_str());
        imageManip->out.link(spatialNode->input);
        setXinXout(pipeline);
    }
    ~SpatialDetection() = default;
    void updateParams(parametersConfig& config) override {
        ph->setRuntimeParams(config);
    };
    void setupQueues(std::shared_ptr<dai::Device> device) override {
        nnQ = device->getOutputQueue(nnQName, ph->getParam<int>("i_max_q_size"), false);
        std::string socketName = utils::getSocketName(static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id")));

        auto tfPrefix = getTFPrefix(socketName);
        int width;
        int height;
        if(ph->getParam<bool>("i_disable_resize")) {
            width = ph->getOtherNodeParam<int>(socketName, "i_preview_width");
            height = ph->getOtherNodeParam<int>(socketName, "i_preview_height");
        } else {
            width = imageManip->initialConfig.getResizeConfig().width;
            height = imageManip->initialConfig.getResizeConfig().height;
        }
        detConverter = std::make_unique<dai::ros::SpatialDetectionConverter>(
            tfPrefix + "_camera_optical_frame", width, height, false, ph->getParam<bool>("i_get_base_device_timestamp"));
        detConverter->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
        nnQ->addCallback(std::bind(&SpatialDetection::spatialCB, this, std::placeholders::_1, std::placeholders::_2));
        detPub = getROSNode().template advertise<vision_msgs::Detection3DArray>(getName() + "/spatial_detections", 10);

        if(ph->getParam<bool>("i_enable_passthrough")) {
            ptQ = device->getOutputQueue(ptQName, ph->getParam<int>("i_max_q_size"), false);
            ptImageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
            ptImageConverter->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
            ptInfoMan = std::make_shared<camera_info_manager::CameraInfoManager>(ros::NodeHandle(getROSNode(), getName()), "/" + getName());
            ptInfoMan->setCameraInfo(sensor_helpers::getCalibInfo(*ptImageConverter, device, dai::CameraBoardSocket::CAM_A, width, height));

            ptPub = it.advertiseCamera(getName() + "/passthrough/image_raw", 1);
            ptQ->addCallback(std::bind(sensor_helpers::basicCameraPub, std::placeholders::_1, std::placeholders::_2, *ptImageConverter, ptPub, ptInfoMan));
        }

        if(ph->getParam<bool>("i_enable_passthrough_depth")) {
            dai::CameraBoardSocket socket = static_cast<dai::CameraBoardSocket>(ph->getOtherNodeParam<int>("stereo", "i_board_socket_id"));
            if(!ph->getOtherNodeParam<bool>("stereo", "i_align_depth")) {
                tfPrefix = getTFPrefix("right");
                socket = dai::CameraBoardSocket::CAM_C;
            };
            ptDepthQ = device->getOutputQueue(ptDepthQName, ph->getParam<int>("i_max_q_size"), false);
            ptDepthImageConverter = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
            ptDepthImageConverter->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
            ptDepthInfoMan = std::make_shared<camera_info_manager::CameraInfoManager>(ros::NodeHandle(getROSNode(), getName()), "/" + getName());
            int width = ph->getOtherNodeParam<int>("stereo", "i_width");
            int height = ph->getOtherNodeParam<int>("stereo", "i_height");
            ptDepthInfoMan->setCameraInfo(sensor_helpers::getCalibInfo(*ptDepthImageConverter, device, socket, width, height));

            ptDepthPub = it.advertiseCamera(getName() + "/passthrough_depth/image_raw", 1);
            ptDepthQ->addCallback(
                std::bind(sensor_helpers::basicCameraPub, std::placeholders::_1, std::placeholders::_2, *ptDepthImageConverter, ptDepthPub, ptDepthInfoMan));
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
        std::deque<vision_msgs::Detection3DArray> deq;
        detConverter->toRosVisionMsg(inDet, deq);
        while(deq.size() > 0) {
            auto currMsg = deq.front();
            detPub.publish(currMsg);
            deq.pop_front();
        }
    };
    std::unique_ptr<dai::ros::SpatialDetectionConverter> detConverter;
    image_transport::ImageTransport it;
    std::vector<std::string> labelNames;
    ros::Publisher detPub;
    std::unique_ptr<dai::ros::ImageConverter> ptImageConverter, ptDepthImageConverter;
    image_transport::CameraPublisher ptPub, ptDepthPub;
    sensor_msgs::CameraInfo ptInfo, ptDepthInfo;
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