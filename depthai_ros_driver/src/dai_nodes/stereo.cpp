#include "depthai_ros_driver/dai_nodes/stereo.hpp"

#include "depthai/device/DeviceBase.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ADatatype.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai_ros_driver/dai_nodes/nn/nn_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/nn/spatial_nn_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/feature_tracker.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "depthai_ros_driver/param_handlers/stereo_param_handler.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
Stereo::Stereo(const std::string& daiNodeName,
               std::shared_ptr<rclcpp::Node> node,
               std::shared_ptr<dai::Pipeline> pipeline,
               std::shared_ptr<dai::Device> device,
               dai::CameraBoardSocket leftSocket,
               dai::CameraBoardSocket rightSocket)
    : BaseNode(daiNodeName, node, pipeline) {
    RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
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
    left = std::make_unique<SensorWrapper>(getSocketName(leftSensInfo.socket), node, pipeline, device, leftSensInfo.socket, false);
    right = std::make_unique<SensorWrapper>(getSocketName(rightSensInfo.socket), node, pipeline, device, rightSensInfo.socket, false);
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

    RCLCPP_DEBUG(getLogger(), "Node %s created", daiNodeName.c_str());
}
Stereo::~Stereo() = default;
void Stereo::setNames() {
    stereoQName = getName() + "_stereo";
    leftRectQName = getName() + "_left_rect";
    rightRectQName = getName() + "_right_rect";
}

void Stereo::setXinXout(std::shared_ptr<dai::Pipeline> pipeline) {
    bool outputDisparity = ph->getParam<bool>("i_output_disparity");
    std::function<void(dai::Node::Input)> stereoLinkChoice;
    if(outputDisparity) {
        stereoLinkChoice = [&](auto input) { stereoCamNode->disparity.link(input); };
    } else {
        stereoLinkChoice = [&](auto input) { stereoCamNode->depth.link(input); };
    }
    if(ph->getParam<bool>("i_publish_topic")) {
        stereoPub = setupOutput(
            pipeline, stereoQName, stereoLinkChoice, ph->getParam<bool>("i_synced"), ph->getParam<bool>("i_low_bandwidth"), ph->getParam<int>("i_quality"));
    }

    if(ph->getParam<bool>("i_publish_left_rect") || ph->getParam<bool>("i_publish_synced_rect_pair")) {
        leftRectPub = setupOutput(
            pipeline,
            leftRectQName,
            [&](auto input) { stereoCamNode->rectifiedLeft.link(input); },
            ph->getParam<bool>("i_left_rect_synced"),
            ph->getParam<bool>("i_left_rect_low_bandwidth"),
            ph->getParam<int>("i_left_rect_low_bandwidth_quality"));
    }

    if(ph->getParam<bool>("i_publish_right_rect") || ph->getParam<bool>("i_publish_synced_rect_pair")) {
        rightRectPub = setupOutput(
            pipeline,
            rightRectQName,
            [&](auto input) { stereoCamNode->rectifiedRight.link(input); },
            ph->getParam<bool>("i_right_rect_synced"),
            ph->getParam<bool>("i_right_rect_low_bandwidth"),
            ph->getParam<int>("i_right_rect_low_bandwidth_quality"));
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
                            std::shared_ptr<sensor_helpers::ImagePublisher> pub,
                            bool isLeft) {
    auto sensorName = getSocketName(sensorInfo.socket);
    auto tfPrefix = getOpticalTFPrefix(sensorName);
    sensor_helpers::ImgConverterConfig convConfig;
    convConfig.tfPrefix = tfPrefix;
    convConfig.interleaved = false;
    convConfig.getBaseDeviceTimestamp = ph->getParam<bool>("i_get_base_device_timestamp");
    convConfig.updateROSBaseTimeOnRosMsg = ph->getParam<bool>("i_update_ros_base_time_on_ros_msg");
    convConfig.lowBandwidth = ph->getParam<bool>(isLeft ? "i_left_rect_low_bandwidth" : "i_right_rect_low_bandwidth");
    convConfig.encoding = dai::RawImgFrame::Type::GRAY8;
    convConfig.addExposureOffset = ph->getParam<bool>(isLeft ? "i_left_rect_add_exposure_offset" : "i_right_rect_add_exposure_offset");
    convConfig.expOffset = static_cast<dai::CameraExposureOffset>(ph->getParam<int>(isLeft ? "i_left_rect_exposure_offset" : "i_right_rect_exposure_offset"));
    convConfig.reverseSocketOrder = ph->getParam<bool>("i_reverse_stereo_socket_order");

    sensor_helpers::ImgPublisherConfig pubConfig;
    pubConfig.daiNodeName = sensorName;
    pubConfig.rectified = true;
    pubConfig.width = ph->getOtherNodeParam<int>(sensorName, "i_width");
    pubConfig.height = ph->getOtherNodeParam<int>(sensorName, "i_height");
    pubConfig.topicName = "~/" + sensorName;
    pubConfig.topicSuffix = "/image_rect";
    pubConfig.maxQSize = ph->getOtherNodeParam<int>(sensorName, "i_max_q_size");
    pubConfig.socket = sensorInfo.socket;
    pubConfig.infoMgrSuffix = "rect";

    pub->setup(device, convConfig, pubConfig);
}

void Stereo::setupLeftRectQueue(std::shared_ptr<dai::Device> device) {
    setupRectQueue(device, leftSensInfo, leftRectPub, true);
}

void Stereo::setupRightRectQueue(std::shared_ptr<dai::Device> device) {
    setupRectQueue(device, rightSensInfo, rightRectPub, false);
}

void Stereo::setupStereoQueue(std::shared_ptr<dai::Device> device) {
    std::string tfPrefix;
    if(ph->getParam<bool>("i_align_depth")) {
        tfPrefix = getOpticalTFPrefix(ph->getParam<std::string>("i_socket_name"));
    } else {
        tfPrefix = getOpticalTFPrefix(getSocketName(rightSensInfo.socket).c_str());
    }
    sensor_helpers::ImgConverterConfig convConfig;
    convConfig.getBaseDeviceTimestamp = ph->getParam<bool>("i_get_base_device_timestamp");
    convConfig.tfPrefix = tfPrefix;
    convConfig.updateROSBaseTimeOnRosMsg = ph->getParam<bool>("i_update_ros_base_time_on_ros_msg");
    convConfig.lowBandwidth = ph->getParam<bool>("i_low_bandwidth");
    convConfig.encoding = dai::RawImgFrame::Type::RAW8;
    convConfig.addExposureOffset = ph->getParam<bool>("i_add_exposure_offset");
    convConfig.expOffset = static_cast<dai::CameraExposureOffset>(ph->getParam<int>("i_exposure_offset"));
    convConfig.reverseSocketOrder = ph->getParam<bool>("i_reverse_stereo_socket_order");
    convConfig.alphaScalingEnabled = ph->getParam<bool>("i_enable_alpha_scaling");
    convConfig.alphaScaling = ph->getParam<double>("i_alpha_scaling");
    convConfig.outputDisparity = ph->getParam<bool>("i_output_disparity");

    sensor_helpers::ImgPublisherConfig pubConf;
    pubConf.daiNodeName = getName();
    pubConf.topicName = "~/" + getName();
    pubConf.rectified = !convConfig.alphaScalingEnabled;
    pubConf.width = ph->getParam<int>("i_width");
    pubConf.height = ph->getParam<int>("i_height");
    pubConf.socket = static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"));
    pubConf.calibrationFile = ph->getParam<std::string>("i_calibration_file");
    pubConf.leftSocket = leftSensInfo.socket;
    pubConf.rightSocket = rightSensInfo.socket;
    pubConf.lazyPub = ph->getParam<bool>("i_enable_lazy_publisher");
    pubConf.maxQSize = ph->getParam<int>("i_max_q_size");

    stereoPub->setup(device, convConfig, pubConf);
}
void Stereo::syncTimerCB() {
    auto left = leftRectQ->get<dai::ImgFrame>();
    auto right = rightRectQ->get<dai::ImgFrame>();
    if(left->getSequenceNum() != right->getSequenceNum()) {
        RCLCPP_WARN(getROSNode()->get_logger(), "Left and right rectified frames are not synchronized!");
    } else {
        leftRectPub->publish(left);
        rightRectPub->publish(right);
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
        RCLCPP_INFO(getROSNode()->get_logger(), "Setting up stereo pair sync timer with period %d ms based on left sensor FPS.", timerPeriod);
        leftRectQ = leftRectPub->getQueue();
        rightRectQ = rightRectPub->getQueue();
        syncTimer = getROSNode()->create_wall_timer(std::chrono::milliseconds(timerPeriod), std::bind(&Stereo::syncTimerCB, this));
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
        stereoPub->closeQueue();
    }
    if(ph->getParam<bool>("i_publish_left_rect")) {
        leftRectPub->closeQueue();
    }
    if(ph->getParam<bool>("i_publish_right_rect")) {
        rightRectPub->closeQueue();
    }
    if(ph->getParam<bool>("i_publish_synced_rect_pair")) {
        syncTimer->cancel();
        leftRectPub->closeQueue();
        rightRectPub->closeQueue();
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

void Stereo::link(dai::Node::Input in, int linkType) {
    if(linkType == static_cast<int>(link_types::StereoLinkType::stereo)) {
        stereoCamNode->depth.link(in);
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
    if(ph->getParam<bool>("i_synced")) {
        pubs.push_back(stereoPub);
    }
    if(ph->getParam<bool>("i_left_rect_synced")) {
        pubs.push_back(leftRectPub);
    }
    if(ph->getParam<bool>("i_right_rect_synced")) {
        pubs.push_back(rightRectPub);
    }
    auto pubsLeft = left->getPublishers();
    pubs.insert(pubs.end(), pubsLeft.begin(), pubsLeft.end());
    auto pubsRight = right->getPublishers();
    pubs.insert(pubs.end(), pubsRight.begin(), pubsRight.end());
    return pubs;
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

void Stereo::updateParams(const std::vector<rclcpp::Parameter>& params) {
    ph->setRuntimeParams(params);
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver
