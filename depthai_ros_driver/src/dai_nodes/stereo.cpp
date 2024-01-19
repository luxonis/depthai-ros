#include "depthai_ros_driver/dai_nodes/stereo.hpp"

#include "camera_info_manager/camera_info_manager.h"
#include "cv_bridge/cv_bridge.h"
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/DeviceBase.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/dai_nodes/nn/nn_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/nn/spatial_nn_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/feature_tracker.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "depthai_ros_driver/param_handlers/stereo_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "image_transport/camera_publisher.h"
#include "image_transport/image_transport.h"
#include "ros/node_handle.h"

namespace depthai_ros_driver {
namespace dai_nodes {
Stereo::Stereo(const std::string& daiNodeName,
               ros::NodeHandle node,
               std::shared_ptr<dai::Pipeline> pipeline,
               std::shared_ptr<dai::Device> device,
               dai::CameraBoardSocket leftSocket,
               dai::CameraBoardSocket rightSocket)
    : BaseNode(daiNodeName, node, pipeline), it(node) {
    ROS_DEBUG("Creating node %s", daiNodeName.c_str());
    setNames();
    ph = std::make_unique<param_handlers::StereoParamHandler>(node, daiNodeName);
    auto alignSocket = dai::CameraBoardSocket::CAM_A;
    if(device->getDeviceName() == "OAK-D-SR") {
        alignSocket = dai::CameraBoardSocket::CAM_C;
    }
    ph->updateSocketsFromParams(leftSocket, rightSocket, alignSocket);
    auto features = device->getConnectedCameraFeatures();
    for(auto f : features) {
        if(f.socket == leftSocket) {
            leftSensInfo = f;
        } else if(f.socket == rightSocket) {
            rightSensInfo = f;
        } else {
            continue;
        }
    }
    ROS_DEBUG("Creating stereo node with left sensor %s and right sensor %s",
              utils::getSocketName(leftSensInfo.socket).c_str(),
              utils::getSocketName(rightSensInfo.socket).c_str());
    left = std::make_unique<SensorWrapper>(utils::getSocketName(leftSensInfo.socket), node, pipeline, device, leftSensInfo.socket, false);
    right = std::make_unique<SensorWrapper>(utils::getSocketName(rightSensInfo.socket), node, pipeline, device, rightSensInfo.socket, false);
    stereoCamNode = pipeline->create<dai::node::StereoDepth>();
    ph->declareParams(stereoCamNode);
    setXinXout(pipeline);
    left->link(stereoCamNode->left);
    right->link(stereoCamNode->right);
    if(ph->getParam<bool>("i_enable_spatial_nn")) {
        if(ph->getParam<std::string>("i_spatial_nn_source") == "left") {
            nnNode = std::make_unique<SpatialNNWrapper>(getName() + "_spatial_nn", getROSNode(), pipeline, leftSensInfo.socket);
            left->link(nnNode->getInput(static_cast<int>(dai_nodes::nn_helpers::link_types::SpatialNNLinkType::input)),
                       static_cast<int>(dai_nodes::link_types::RGBLinkType::preview));
        } else {
            nnNode = std::make_unique<SpatialNNWrapper>(getName() + "_spatial_nn", getROSNode(), pipeline, rightSensInfo.socket);
            right->link(nnNode->getInput(static_cast<int>(dai_nodes::nn_helpers::link_types::SpatialNNLinkType::input)),
                        static_cast<int>(dai_nodes::link_types::RGBLinkType::preview));
        }
        stereoCamNode->depth.link(nnNode->getInput(static_cast<int>(dai_nodes::nn_helpers::link_types::SpatialNNLinkType::inputDepth)));
    }
    ROS_DEBUG("Node %s created", daiNodeName.c_str());
}
Stereo::~Stereo() = default;
void Stereo::setNames() {
    stereoQName = getName() + "_stereo";
    leftRectQName = getName() + "_left_rect";
    rightRectQName = getName() + "_right_rect";
}

void Stereo::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    if(ph->getParam<bool>("i_publish_topic")) {
        xoutStereo = pipeline->create<dai::node::XLinkOut>();
        xoutStereo->setStreamName(stereoQName);
        xoutStereo->input.setBlocking(false);
        if(ph->getParam<bool>("i_low_bandwidth")) {
            stereoEnc = sensor_helpers::createEncoder(pipeline, ph->getParam<int>("i_low_bandwidth_quality"));
            stereoCamNode->disparity.link(stereoEnc->input);
            stereoEnc->bitstream.link(xoutStereo->input);
        } else {
            if(ph->getParam<bool>("i_output_disparity")) {
                stereoCamNode->disparity.link(xoutStereo->input);
            } else {
                stereoCamNode->depth.link(xoutStereo->input);
            }
        }
    }
    if(ph->getParam<bool>("i_publish_left_rect") || ph->getParam<bool>("i_publish_synced_rect_pair")) {
        xoutLeftRect = pipeline->create<dai::node::XLinkOut>();
        xoutLeftRect->setStreamName(leftRectQName);
        if(ph->getParam<bool>("i_left_rect_low_bandwidth")) {
            leftRectEnc = sensor_helpers::createEncoder(pipeline, ph->getParam<int>("i_left_rect_low_bandwidth_quality"));
            stereoCamNode->rectifiedLeft.link(leftRectEnc->input);
            leftRectEnc->bitstream.link(xoutLeftRect->input);
        } else {
            stereoCamNode->rectifiedLeft.link(xoutLeftRect->input);
        }
    }

    if(ph->getParam<bool>("i_publish_right_rect") || ph->getParam<bool>("i_publish_synced_rect_pair")) {
        xoutRightRect = pipeline->create<dai::node::XLinkOut>();
        xoutRightRect->setStreamName(rightRectQName);
        if(ph->getParam<bool>("i_right_rect_low_bandwidth")) {
            rightRectEnc = sensor_helpers::createEncoder(pipeline, ph->getParam<int>("i_right_rect_low_bandwidth_quality"));
            stereoCamNode->rectifiedRight.link(rightRectEnc->input);
            rightRectEnc->bitstream.link(xoutRightRect->input);
        } else {
            stereoCamNode->rectifiedRight.link(xoutRightRect->input);
        }
    }

    if(ph->getParam<bool>("i_left_rect_enable_feature_tracker")) {
        featureTrackerLeftR = std::make_unique<FeatureTracker>(leftSensInfo.name + std::string("_rect_feature_tracker"), getROSNode(), pipeline);

        stereoCamNode->rectifiedLeft.link(featureTrackerLeftR->getInput());
    }

    if(ph->getParam<bool>("i_right_rect_enable_feature_tracker")) {
        featureTrackerRightR = std::make_unique<FeatureTracker>(rightSensInfo.name + std::string("_rect_feature_tracker"), getROSNode(), pipeline);
        stereoCamNode->rectifiedRight.link(featureTrackerRightR->getInput());
    }
}

void Stereo::setupRectQueue(std::shared_ptr<dai::Device> device,
                            dai::CameraFeatures& sensorInfo,
                            const std::string& queueName,
                            std::unique_ptr<dai::ros::ImageConverter>& conv,
                            std::shared_ptr<camera_info_manager::CameraInfoManager>& im,
                            std::shared_ptr<dai::DataOutputQueue>& q,
                            image_transport::CameraPublisher& pubIT,
                            bool isLeft) {
    auto sensorName = utils::getSocketName(sensorInfo.socket);
    auto tfPrefix = getTFPrefix(sensorName);
    conv = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false, ph->getParam<bool>("i_get_base_device_timestamp"));
    conv->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
    bool lowBandwidth = ph->getParam<bool>(isLeft ? "i_left_rect_low_bandwidth" : "i_right_rect_low_bandwidth");
    if(lowBandwidth) {
        conv->convertFromBitstream(dai::RawImgFrame::Type::GRAY8);
    }
    bool addOffset = ph->getParam<bool>(isLeft ? "i_left_rect_add_exposure_offset" : "i_right_rect_add_exposure_offset");
    if(addOffset) {
        auto offset = static_cast<dai::CameraExposureOffset>(ph->getParam<int>(isLeft ? "i_left_rect_exposure_offset" : "i_right_rect_exposure_offset"));
        conv->addExposureOffset(offset);
    }
    im = std::make_shared<camera_info_manager::CameraInfoManager>(ros::NodeHandle(getROSNode(), sensorName), "/" + sensorName + "/rect");
    if(ph->getParam<bool>("i_reverse_stereo_socket_order")) {
        conv->reverseStereoSocketOrder();
    }
    auto info = sensor_helpers::getCalibInfo(
        *conv, device, sensorInfo.socket, ph->getOtherNodeParam<int>(sensorName, "i_width"), ph->getOtherNodeParam<int>(sensorName, "i_height"));
    for(auto& d : info.D) {
        d = 0.0;
    }

    for(auto& r : info.R) {
        r = 0.0;
    }
    info.R[0] = info.R[4] = info.R[8] = 1.0;

    im->setCameraInfo(info);

    q = device->getOutputQueue(queueName, ph->getOtherNodeParam<int>(sensorName, "i_max_q_size"), false);

    // if publish synced pair is set to true then we skip individual publishing of left and right rectified frames
    bool addCallback = !ph->getParam<bool>("i_publish_synced_rect_pair");

    pubIT = it.advertiseCamera(sensorName + "/image_rect", 1);
    if(addCallback) {
        q->addCallback(std::bind(
            sensor_helpers::cameraPub, std::placeholders::_1, std::placeholders::_2, *conv, pubIT, im, ph->getParam<bool>("i_enable_lazy_publisher")));
    }
}

void Stereo::setupLeftRectQueue(std::shared_ptr<dai::Device> device) {
    setupRectQueue(device, leftSensInfo, leftRectQName, leftRectConv, leftRectIM, leftRectQ, leftRectPubIT, true);
}

void Stereo::setupRightRectQueue(std::shared_ptr<dai::Device> device) {
    setupRectQueue(device, rightSensInfo, rightRectQName, rightRectConv, rightRectIM, rightRectQ, rightRectPubIT, false);
}

void Stereo::setupStereoQueue(std::shared_ptr<dai::Device> device) {
    stereoQ = device->getOutputQueue(stereoQName, ph->getParam<int>("i_max_q_size"), false);
    std::string tfPrefix;
    if(ph->getParam<bool>("i_align_depth")) {
        tfPrefix = getTFPrefix(ph->getParam<std::string>("i_socket_name"));
    } else {
        tfPrefix = getTFPrefix(utils::getSocketName(rightSensInfo.socket));
    }
    stereoConv = std::make_unique<dai::ros::ImageConverter>(tfPrefix + "_camera_optical_frame", false);
    stereoConv->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
    if(ph->getParam<bool>("i_low_bandwidth")) {
        stereoConv->convertFromBitstream(dai::RawImgFrame::Type::RAW8);
    }

    if(ph->getParam<bool>("i_add_exposure_offset")) {
        auto offset = static_cast<dai::CameraExposureOffset>(ph->getParam<int>("i_exposure_offset"));
        stereoConv->addExposureOffset(offset);
    }

    if(ph->getParam<bool>("i_reverse_stereo_socket_order")) {
        stereoConv->reverseStereoSocketOrder();
    }
    if(ph->getParam<bool>("i_enable_alpha_scaling")) {
        stereoConv->setAlphaScaling(ph->getParam<double>("i_alpha_scaling"));
    }
    stereoIM = std::make_shared<camera_info_manager::CameraInfoManager>(ros::NodeHandle(getROSNode(), getName()), "/" + getName());
    auto info = sensor_helpers::getCalibInfo(*stereoConv,
                                             device,
                                             static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id")),
                                             ph->getParam<int>("i_width"),
                                             ph->getParam<int>("i_height"));
    auto calibHandler = device->readCalibration();
    if(!ph->getParam<bool>("i_output_disparity")) {
        if(ph->getParam<bool>("i_reverse_stereo_socket_order")) {
            stereoConv->convertDispToDepth(calibHandler.getBaselineDistance(leftSensInfo.socket, rightSensInfo.socket, false));
        } else {
            stereoConv->convertDispToDepth(calibHandler.getBaselineDistance(rightSensInfo.socket, leftSensInfo.socket, false));
        }
    }
    // remove distortion if alpha scaling is not enabled
    if(!ph->getParam<bool>("i_enable_alpha_scaling")) {
        for(auto& d : info.D) {
            d = 0.0;
        }
        for(auto& r : info.R) {
            r = 0.0;
        }
        info.R[0] = info.R[4] = info.R[8] = 1.0;
    }
    stereoIM->setCameraInfo(info);

    stereoPubIT = it.advertiseCamera(getName() + "/image_raw", 1);
    stereoQ->addCallback(std::bind(sensor_helpers::cameraPub,
                                   std::placeholders::_1,
                                   std::placeholders::_2,
                                   *stereoConv,
                                   stereoPubIT,
                                   stereoIM,
                                   ph->getParam<bool>("i_enable_lazy_publisher")));
}

void Stereo::syncTimerCB() {
    auto left = leftRectQ->get<dai::ImgFrame>();
    auto right = rightRectQ->get<dai::ImgFrame>();
    if(left->getSequenceNum() != right->getSequenceNum()) {
        ROS_WARN("Left and right rectified frames are not synchronized!");
    } else {
        bool lazyPub = ph->getParam<bool>("i_enable_lazy_publisher");
        if(ros::ok() && (!lazyPub || leftRectPubIT.getNumSubscribers() > 0 || rightRectPubIT.getNumSubscribers() > 0)) {
            auto leftInfo = leftRectIM->getCameraInfo();
            auto leftRawMsg = leftRectConv->toRosMsgRawPtr(left);
            leftInfo.header = leftRawMsg.header;
            auto rightInfo = rightRectIM->getCameraInfo();
            auto rightRawMsg = rightRectConv->toRosMsgRawPtr(right);
            rightRawMsg.header.stamp = leftRawMsg.header.stamp;
            rightInfo.header = rightRawMsg.header;
            leftRectPubIT.publish(leftRawMsg, leftInfo);
            rightRectPubIT.publish(rightRawMsg, rightInfo);
        }
    }
}

void Stereo::setupQueues(std::shared_ptr<dai::Device> device) {
    left->setupQueues(device);
    right->setupQueues(device);
    if(ph->getParam<bool>("i_publish_topic")) {
        setupStereoQueue(device);
    }
    if(ph->getParam<bool>("i_publish_left_rect") || ph->getParam<bool>("i_publish_synced_rect_pair")) {
        setupLeftRectQueue(device);
    }
    if(ph->getParam<bool>("i_publish_right_rect") || ph->getParam<bool>("i_publish_synced_rect_pair")) {
        setupRightRectQueue(device);
    }
    if(ph->getParam<bool>("i_publish_synced_rect_pair")) {
        int timerPeriod = 1000.0 / ph->getOtherNodeParam<double>(leftSensInfo.name, "i_fps");
        ROS_INFO("Setting up stereo pair sync timer with period %d ms based on left sensor FPS.", timerPeriod);
        syncTimer = std::make_shared<ros::Timer>(getROSNode().createTimer(ros::Duration(1.0 / timerPeriod), std::bind(&Stereo::syncTimerCB, this)));
    }
    if(ph->getParam<bool>("i_left_rect_enable_feature_tracker")) {
        featureTrackerLeftR->setupQueues(device);
    }
    if(ph->getParam<bool>("i_right_rect_enable_feature_tracker")) {
        featureTrackerRightR->setupQueues(device);
    }
    if(ph->getParam<bool>("i_enable_spatial_nn")) {
        nnNode->setupQueues(device);
    }
}
void Stereo::closeQueues() {
    left->closeQueues();
    right->closeQueues();
    if(ph->getParam<bool>("i_publish_topic")) {
        stereoQ->close();
    }
    if(ph->getParam<bool>("i_publish_left_rect")) {
        leftRectQ->close();
    }
    if(ph->getParam<bool>("i_publish_right_rect")) {
        rightRectQ->close();
    }
    if(ph->getParam<bool>("i_publish_synced_rect_pair")) {
        syncTimer.reset();
        leftRectQ->close();
        rightRectQ->close();
    }
    if(ph->getParam<bool>("i_left_rect_enable_feature_tracker")) {
        featureTrackerLeftR->closeQueues();
    }
    if(ph->getParam<bool>("i_right_rect_enable_feature_tracker")) {
        featureTrackerRightR->closeQueues();
    }
    if(ph->getParam<bool>("i_enable_spatial_nn")) {
        nnNode->closeQueues();
    }
}

void Stereo::link(dai::Node::Input in, int /*linkType*/) {
    stereoCamNode->depth.link(in);
}

dai::Node::Input Stereo::getInput(int linkType) {
    if(linkType == static_cast<int>(link_types::StereoLinkType::left)) {
        return stereoCamNode->left;
    } else if(linkType == static_cast<int>(link_types::StereoLinkType::right)) {
        return stereoCamNode->right;
    } else {
        throw std::runtime_error("Wrong link type specified!");
    }
}

void Stereo::updateParams(parametersConfig& config) {
    ph->setRuntimeParams(config);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
