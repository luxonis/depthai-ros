#include "depthai_ros_driver/dai_nodes/sensors/stereo.hpp"

#include <depthai/capabilities/ImgFrameCapability.hpp>
#include <optional>

#include "depthai/device/DeviceBase.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai_ros_driver/dai_nodes/nn/nn_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/nn/spatial_nn_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/feature_tracker.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/rgbd.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "depthai_ros_driver/param_handlers/stereo_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
Stereo::Stereo(const std::string& daiNodeName,
               std::shared_ptr<rclcpp::Node> node,
               std::shared_ptr<dai::Pipeline> pipeline,
               std::shared_ptr<dai::Device> device,
               bool rsCompat,
               dai::CameraBoardSocket leftSocket,
               dai::CameraBoardSocket rightSocket)
    : BaseNode(daiNodeName, node, pipeline, device->getDeviceName(), rsCompat) {
    using ParamNames = param_handlers::ParamNames;
    RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
    setNames();
    ph = std::make_unique<param_handlers::StereoParamHandler>(node, daiNodeName, device->getDeviceName(), rsCompat);
    auto alignSocket = dai::CameraBoardSocket::CAM_A;
    if(device->getDeviceName() == "OAK-D-SR" || device->getDeviceName() == "OAK-D-SR-POE") {
        alignSocket = dai::CameraBoardSocket::CAM_C;
    }
    ph->updateSocketsFromParams(leftSocket, rightSocket, alignSocket);
    auto features = device->getConnectedCameraFeatures();
    for(auto f : features) {
        if(f.socket == leftSocket) {
            leftSensInfo = f;
            leftSensInfo.name = getSocketName(leftSocket);
        } else if(f.socket == rightSocket) {
            rightSensInfo = f;
            rightSensInfo.name = getSocketName(rightSocket);
        } else {
            continue;
        }
    }
    RCLCPP_DEBUG(getLogger(),
                 "Creating stereo node with left sensor %s and right sensor %s",
                 getSocketName(leftSensInfo.socket).c_str(),
                 getSocketName(rightSensInfo.socket).c_str());
    left = std::make_shared<SensorWrapper>(getSocketName(leftSensInfo.socket), node, pipeline, device->getDeviceName(), rsCompat, leftSensInfo.socket, false);
    right =
        std::make_shared<SensorWrapper>(getSocketName(rightSensInfo.socket), node, pipeline, device->getDeviceName(), rsCompat, rightSensInfo.socket, false);
    stereoCamNode = pipeline->create<dai::node::StereoDepth>();
    ph->declareParams(stereoCamNode);
    leftOut = left->getUnderlyingNode()->requestOutput(std::make_pair<int, int>(ph->getParam<int>(ParamNames::WIDTH), ph->getParam<int>(ParamNames::HEIGHT)),
                                                       std::nullopt,
                                                       dai::ImgResizeMode::CROP,
                                                       ph->getParam<float>(ParamNames::FPS));
    rightOut = right->getUnderlyingNode()->requestOutput(std::make_pair<int, int>(ph->getParam<int>(ParamNames::WIDTH), ph->getParam<int>(ParamNames::HEIGHT)),
                                                         std::nullopt,
                                                         dai::ImgResizeMode::CROP,
                                                         ph->getParam<float>(ParamNames::FPS));
    leftOut->link(stereoCamNode->left);
    rightOut->link(stereoCamNode->right);

    aligned = ph->getParam<bool>(param_handlers::ParamNames::ALIGNED);
    if(ph->getParam<bool>("i_enable_left_spatial_nn")) {
        nnNodeLeft = std::make_unique<SpatialNNWrapper>(
            getName() + "_" + left->getName() + "_spatial_nn", getROSNode(), pipeline, device->getDeviceName(), rsCompat, *left, *this);
    }
    if(ph->getParam<bool>("i_enable_right_spatial_nn")) {
        nnNodeRight = std::make_unique<SpatialNNWrapper>(
            getName() + "_" + right->getName() + "_spatial_nn", getROSNode(), pipeline, device->getDeviceName(), rsCompat, *right, *this);
    }
    if(ph->getParam<bool>("i_enable_left_rgbd")) {
        rgbdNodeLeft = std::make_unique<dai_nodes::RGBD>(
            getName() + "_" + left->getName() + "_rgbd", node, pipeline, device, rsCompat, *left, getUnderlyingNode(), aligned);
    }
    if(ph->getParam<bool>("i_enable_right_rgbd")) {
        rgbdNodeRight = std::make_unique<dai_nodes::RGBD>(
            getName() + "_" + right->getName() + "_rgbd", node, pipeline, device, rsCompat, *right, getUnderlyingNode(), aligned);
    }

    // Check alignment, if board socket is one of the pairs, align.
    // if not it should be aligned externally by calling align method in pipeline creation
    auto socketID = ph->getSocketID();
    platform = device->getPlatform();
    if(aligned) {
        if(platform == dai::Platform::RVC4) {
            alignNode = pipeline->create<dai::node::ImageAlign>();
            alignNode->setRunOnHost(ph->getParam<bool>("i_run_align_on_host"));
            stereoCamNode->depth.link(alignNode->input);
            alignNode->input.setBlocking(false);
            alignNode->inputAlignTo.setBlocking(false);
        }
        if(socketID == leftSensInfo.socket) {
            leftOut->link(getInput(static_cast<int>(link_types::StereoLinkType::align)));
        } else if(socketID == rightSensInfo.socket) {
            rightOut->link(getInput(static_cast<int>(link_types::StereoLinkType::align)));
        } else {
            RCLCPP_DEBUG(getLogger(), "Socket aligned to a different ID: %d, make sure you call align method in pipeline creation", static_cast<int>(socketID));
        }
    }
    setInOut(pipeline);
    RCLCPP_DEBUG(getLogger(), "Node %s created", daiNodeName.c_str());
}
Stereo::~Stereo() = default;
void Stereo::setNames() {
    stereoQName = getName() + "_stereo";
    leftRectQName = getName() + "_left_rect";
    rightRectQName = getName() + "_right_rect";
}

std::shared_ptr<dai::node::StereoDepth> Stereo::getUnderlyingNode() {
    return stereoCamNode;
}

bool Stereo::isAligned() {
    return aligned;
}

void Stereo::setInOut(std::shared_ptr<dai::Pipeline> pipeline) {
    bool outputDisparity = ph->getParam<bool>("i_output_disparity");
    bool lowBandwidth = ph->getParam<bool>("i_low_bandwidth");
    if(ph->getParam<bool>("i_publish_topic")) {
        utils::VideoEncoderConfig encConf;
        encConf.profile = static_cast<dai::VideoEncoderProperties::Profile>(ph->getParam<int>("i_low_bandwidth_profile"));
        encConf.bitrate = ph->getParam<int>("i_low_bandwidth_bitrate");
        encConf.frameFreq = ph->getParam<int>("i_low_bandwidth_frame_freq");
        encConf.quality = ph->getParam<int>("i_low_bandwidth_quality");
        encConf.enabled = lowBandwidth;

        if(outputDisparity || lowBandwidth) {
            stereoPub = setupOutput(pipeline, stereoQName, &stereoCamNode->disparity, ph->getParam<bool>("i_synced"), encConf);
        } else {
            if(aligned && platform == dai::Platform::RVC4) {
                stereoPub = setupOutput(pipeline, stereoQName, &alignNode->outputAligned, ph->getParam<bool>("i_synced"), encConf);
            } else {
                stereoPub = setupOutput(pipeline, stereoQName, &stereoCamNode->depth, ph->getParam<bool>("i_synced"), encConf);
            }
        }
    }

    if(ph->getParam<bool>("i_left_rect_publish_topic")) {
        utils::VideoEncoderConfig encConf;
        encConf.profile = static_cast<dai::VideoEncoderProperties::Profile>(ph->getParam<int>("i_left_rect_low_bandwidth_profile"));
        encConf.bitrate = ph->getParam<int>("i_left_rect_low_bandwidth_bitrate");
        encConf.frameFreq = ph->getParam<int>("i_left_rect_low_bandwidth_frame_freq");
        encConf.quality = ph->getParam<int>("i_left_rect_low_bandwidth_quality");
        encConf.enabled = ph->getParam<bool>("i_left_rect_low_bandwidth");

        leftRectPub = setupOutput(pipeline, leftRectQName, &stereoCamNode->rectifiedLeft, ph->getParam<bool>("i_left_rect_synced"), encConf);
    }

    if(ph->getParam<bool>("i_right_rect_publish_topic")) {
        utils::VideoEncoderConfig encConf;
        encConf.profile = static_cast<dai::VideoEncoderProperties::Profile>(ph->getParam<int>("i_right_rect_low_bandwidth_profile"));
        encConf.bitrate = ph->getParam<int>("i_right_rect_low_bandwidth_bitrate");
        encConf.frameFreq = ph->getParam<int>("i_right_rect_low_bandwidth_frame_freq");
        encConf.quality = ph->getParam<int>("i_right_rect_low_bandwidth_quality");
        encConf.enabled = ph->getParam<bool>("i_right_rect_low_bandwidth");
        rightRectPub = setupOutput(pipeline, rightRectQName, &stereoCamNode->rectifiedRight, ph->getParam<bool>("i_right_rect_synced"), encConf);
    }

    if(ph->getParam<bool>("i_left_rect_enable_feature_tracker")) {
        featureTrackerLeftR = std::make_unique<FeatureTracker>(
            leftSensInfo.name + std::string("_rect_feature_tracker"), getROSNode(), pipeline, getDeviceName(), rsCompatibilityMode());
        auto in = featureTrackerLeftR->getInput();
        stereoCamNode->rectifiedLeft.link(in);
    }

    if(ph->getParam<bool>("i_right_rect_enable_feature_tracker")) {
        featureTrackerRightR = std::make_unique<FeatureTracker>(
            rightSensInfo.name + std::string("_rect_feature_tracker"), getROSNode(), pipeline, getDeviceName(), rsCompatibilityMode());
        auto in = featureTrackerRightR->getInput();
        stereoCamNode->rectifiedRight.link(in);
    }
}

void Stereo::setupRectQueue(std::shared_ptr<dai::Device> device,
                            dai::CameraFeatures& sensorInfo,
                            std::shared_ptr<sensor_helpers::ImagePublisher> pub,
                            bool isLeft) {
    auto sensorName = getSocketName(sensorInfo.socket);
    auto tfPrefix = getOpticalFrameName(sensorName);
    utils::ImgConverterConfig convConfig;
    convConfig.tfPrefix = tfPrefix;
    convConfig.interleaved = false;
    convConfig.getBaseDeviceTimestamp = ph->getParam<bool>("i_get_base_device_timestamp");
    convConfig.updateROSBaseTimeOnRosMsg = ph->getParam<bool>("i_update_ros_base_time_on_ros_msg");
    convConfig.lowBandwidth = ph->getParam<bool>(isLeft ? "i_left_rect_low_bandwidth" : "i_right_rect_low_bandwidth");
    convConfig.encoding = dai::ImgFrame::Type::GRAY8;
    convConfig.addExposureOffset = ph->getParam<bool>(isLeft ? "i_left_rect_add_exposure_offset" : "i_right_rect_add_exposure_offset");
    convConfig.expOffset = static_cast<dai::CameraExposureOffset>(ph->getParam<int>(isLeft ? "i_left_rect_exposure_offset" : "i_right_rect_exposure_offset"));
    convConfig.reverseSocketOrder = ph->getParam<bool>("i_reverse_stereo_socket_order");

    utils::ImgPublisherConfig pubConfig;
    pubConfig.daiNodeName = sensorName;
    pubConfig.rectified = true;
    pubConfig.undistorted = true;
    pubConfig.width = ph->getOtherNodeParam<int>(sensorName, "i_width");
    pubConfig.height = ph->getOtherNodeParam<int>(sensorName, "i_height");
    pubConfig.topicName = "~/" + sensorName;
    pubConfig.topicSuffix = rsCompatibilityMode() ? "/image_rect_raw" : "/image_rect";
    pubConfig.maxQSize = ph->getOtherNodeParam<int>(sensorName, "i_max_q_size");
    pubConfig.socket = sensorInfo.socket;
    pubConfig.infoMgrSuffix = "rect";
    pubConfig.publishCompressed = ph->getParam<bool>(isLeft ? "i_left_rect_publish_compressed" : "i_right_rect_publish_compressed");

    pub->setup(device, convConfig, pubConfig);
}

void Stereo::setupLeftRectQueue(std::shared_ptr<dai::Device> device) {
    setupRectQueue(device, leftSensInfo, leftRectPub, true);
}

void Stereo::setupRightRectQueue(std::shared_ptr<dai::Device> device) {
    setupRectQueue(device, rightSensInfo, rightRectPub, false);
}

void Stereo::setupStereoQueue(std::shared_ptr<dai::Device> device) {
    using param_handlers::ParamNames;
    std::string tfPrefix;
    tfPrefix = getOpticalFrameName(ph->getParam<std::string>("i_socket_name"));
    utils::ImgConverterConfig convConfig;
    convConfig.getBaseDeviceTimestamp = ph->getParam<bool>(ParamNames::GET_BASE_DEVICE_TIMESTAMP);
    convConfig.tfPrefix = tfPrefix;
    convConfig.updateROSBaseTimeOnRosMsg = ph->getParam<bool>(ParamNames::UPDATE_ROS_BASE_TIME_ON_ROS_MSG);
    convConfig.lowBandwidth = ph->getParam<bool>(ParamNames::LOW_BANDWIDTH);
    convConfig.encoding = dai::ImgFrame::Type::RAW8;
    convConfig.addExposureOffset = ph->getParam<bool>(ParamNames::ADD_EXPOSURE_OFFSET);
    convConfig.expOffset = static_cast<dai::CameraExposureOffset>(ph->getParam<int>(ParamNames::EXPOSURE_OFFSET));
    convConfig.reverseSocketOrder = ph->getParam<bool>(ParamNames::REVERSE_STEREO_SOCKET_ORDER);
    convConfig.alphaScalingEnabled = ph->getParam<bool>("i_enable_alpha_scaling");
    if(convConfig.alphaScalingEnabled) {
        convConfig.alphaScaling = ph->getParam<double>("i_alpha_scaling");
    }
    convConfig.outputDisparity = ph->getParam<bool>("i_output_disparity");
    convConfig.isStereo = true;

    utils::ImgPublisherConfig pubConf;
    pubConf.daiNodeName = getName();
    pubConf.topicName = "~/" + getName();
    pubConf.topicSuffix = rsCompatibilityMode() ? "/image_rect_raw" : "/image_raw";
    pubConf.rectified = !convConfig.alphaScalingEnabled;
    pubConf.undistorted = !convConfig.alphaScalingEnabled;
    pubConf.width = ph->getParam<int>(ParamNames::WIDTH);
    pubConf.height = ph->getParam<int>(ParamNames::HEIGHT);
    pubConf.socket = ph->getSocketID();
    pubConf.calibrationFile = ph->getParam<std::string>(ParamNames::CALIBRATION_FILE);
    pubConf.leftSocket = leftSensInfo.socket;
    pubConf.rightSocket = rightSensInfo.socket;
    pubConf.lazyPub = ph->getParam<bool>(ParamNames::ENABLE_LAZY_PUBLISHER);
    pubConf.maxQSize = ph->getParam<int>(ParamNames::MAX_Q_SIZE);
    pubConf.publishCompressed = ph->getParam<bool>(ParamNames::PUBLISH_COMPRESSED);

    stereoPub->setup(device, convConfig, pubConf);
}

void Stereo::setupQueues(std::shared_ptr<dai::Device> device) {
    left->setupQueues(device);
    right->setupQueues(device);
    if(ph->getParam<bool>("i_publish_topic")) {
        setupStereoQueue(device);
    }
    if(ph->getParam<bool>("i_enable_left_rgbd")) {
        rgbdNodeLeft->setupQueues(device);
    }
    if(ph->getParam<bool>("i_enable_right_rgbd")) {
        rgbdNodeRight->setupQueues(device);
    }
    if(ph->getParam<bool>("i_left_rect_publish_topic")) {
        setupLeftRectQueue(device);
    }
    if(ph->getParam<bool>("i_right_rect_publish_topic")) {
        setupRightRectQueue(device);
    }
    if(ph->getParam<bool>("i_left_rect_enable_feature_tracker")) {
        featureTrackerLeftR->setupQueues(device);
    }
    if(ph->getParam<bool>("i_right_rect_enable_feature_tracker")) {
        featureTrackerRightR->setupQueues(device);
    }
    if(ph->getParam<bool>("i_enable_left_spatial_nn")) {
        nnNodeLeft->setupQueues(device);
    }
    if(ph->getParam<bool>("i_enable_right_spatial_nn")) {
        nnNodeRight->setupQueues(device);
    }
}
void Stereo::closeQueues() {
    left->closeQueues();
    right->closeQueues();
    if(ph->getParam<bool>("i_publish_topic")) {
        stereoPub->closeQueue();
    }
    if(ph->getParam<bool>("i_enable_left_rgbd")) {
        rgbdNodeLeft->closeQueues();
    }
    if(ph->getParam<bool>("i_enable_right_rgbd")) {
        rgbdNodeRight->closeQueues();
    }
    if(ph->getParam<bool>("i_left_rect_publish_topic")) {
        leftRectPub->closeQueue();
    }
    if(ph->getParam<bool>("i_right_rect_publish_topic")) {
        rightRectPub->closeQueue();
    }
    if(ph->getParam<bool>("i_left_rect_enable_feature_tracker")) {
        featureTrackerLeftR->closeQueues();
    }
    if(ph->getParam<bool>("i_right_rect_enable_feature_tracker")) {
        featureTrackerRightR->closeQueues();
    }
    if(ph->getParam<bool>("i_enable_left_spatial_nn")) {
        nnNodeLeft->closeQueues();
    }
    if(ph->getParam<bool>("i_enable_right_spatial_nn")) {
        nnNodeRight->closeQueues();
    }
}

void Stereo::link(dai::Node::Input& in, int linkType) {
    if(linkType == static_cast<int>(link_types::StereoLinkType::stereo)) {
        if(aligned && platform == dai::Platform::RVC4) {
            alignNode->outputAligned.link(in);
        } else {
            stereoCamNode->depth.link(in);
        }
    } else if(linkType == static_cast<int>(link_types::StereoLinkType::left)) {
        stereoCamNode->rectifiedLeft.link(in);
    } else if(linkType == static_cast<int>(link_types::StereoLinkType::right)) {
        stereoCamNode->rectifiedRight.link(in);
    } else {
        throw std::runtime_error("Wrong link type specified!");
    }
}

std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> Stereo::getPublishers() {
    std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> pubs;
    if(ph->getParam<bool>("i_publish_topic") && ph->getParam<bool>("i_synced")) {
        pubs.push_back(stereoPub);
    }
    if(ph->getParam<bool>("i_left_rect_publish_topic") && ph->getParam<bool>("i_left_rect_synced")) {
        pubs.push_back(leftRectPub);
    }
    if(ph->getParam<bool>("i_right_rect_publish_topic") && ph->getParam<bool>("i_right_rect_synced")) {
        pubs.push_back(rightRectPub);
    }
    auto pubsLeft = left->getPublishers();
    if(!pubsLeft.empty()) {
        pubs.insert(pubs.end(), pubsLeft.begin(), pubsLeft.end());
    }
    auto pubsRight = right->getPublishers();
    if(!pubsRight.empty()) {
        pubs.insert(pubs.end(), pubsRight.begin(), pubsRight.end());
    }
    return pubs;
}
dai::CameraBoardSocket Stereo::getSocketID() {
    return ph->getSocketID();
}

dai::Node::Input& Stereo::getInput(int linkType) {
    if(linkType == static_cast<int>(link_types::StereoLinkType::left)) {
        return stereoCamNode->left;
    } else if(linkType == static_cast<int>(link_types::StereoLinkType::right)) {
        return stereoCamNode->right;
    } else if(linkType == static_cast<int>(link_types::StereoLinkType::align)) {
        if(platform == dai::Platform::RVC2) {
            return stereoCamNode->inputAlignTo;
        } else {
            return alignNode->inputAlignTo;
        }
    } else {
        throw std::runtime_error("Wrong link type specified!");
    }
}
std::shared_ptr<SensorWrapper> Stereo::getLeftSensor() {
    return left;
}
std::shared_ptr<SensorWrapper> Stereo::getRightSensor() {
    return right;
}

int Stereo::getWidth() {
    return ph->getParam<int>(param_handlers::ParamNames::WIDTH);
}
int Stereo::getHeight() {
    return ph->getParam<int>(param_handlers::ParamNames::HEIGHT);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
