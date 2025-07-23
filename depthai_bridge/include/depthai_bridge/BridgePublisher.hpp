#pragma once
#include <deque>
#include <thread>
#include <type_traits>
#include <typeinfo>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/pipeline/MessageQueue.hpp"
#include "ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace depthai_bridge {

namespace ImageMsgs = sensor_msgs::msg;
namespace FFMPEGMsgs = ffmpeg_image_transport_msgs::msg;
using ImagePtr = ImageMsgs::Image::SharedPtr;
using FFMPEGImagePtr = FFMPEGMsgs::FFMPEGPacket::SharedPtr;

template <class RosMsg, class DaiMsg>
class BridgePublisher {
   public:
    using ConvertFunc = std::function<void(std::shared_ptr<DaiMsg>, std::deque<RosMsg>&)>;
    using CustomPublisher = typename std::conditional<std::is_same<RosMsg, ImageMsgs::Image>::value,
                                                      std::shared_ptr<image_transport::CameraPublisher>,
                                                      typename rclcpp::Publisher<RosMsg>::SharedPtr>::type;

    /**
     * @brief Constructor for BridgePublisher with default QoS settings.
     *
     * @param daiMessageQueue Shared pointer to the DepthAI message queue.
     * @param node Shared pointer to the ROS 2 node.
     * @param rosTopic The ROS topic to publish messages to.
     * @param converter Function to convert DepthAI messages to ROS messages.
     * @param qosSetting QoS settings for the ROS publisher.
     * @param lazyPublisher Flag to enable lazy publishing.
     */
    BridgePublisher(std::shared_ptr<dai::MessageQueue> daiMessageQueue,
                    std::shared_ptr<rclcpp::Node> node,
                    std::string rosTopic,
                    ConvertFunc converter,
                    rclcpp::QoS qosSetting = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                    bool lazyPublisher = true);

    /**
     * @brief Constructor for BridgePublisher with custom QoS history depth.
     *
     * @param daiMessageQueue Shared pointer to the DepthAI message queue.
     * @param node Shared pointer to the ROS 2 node.
     * @param rosTopic The ROS topic to publish messages to.
     * @param converter Function to convert DepthAI messages to ROS messages.
     * @param qosHistoryDepth History depth for the QoS settings.
     * @param cameraParamUri URI for the camera parameters.
     * @param cameraName Name of the camera.
     * @param lazyPublisher Flag to enable lazy publishing.
     */
    BridgePublisher(std::shared_ptr<dai::MessageQueue> daiMessageQueue,
                    std::shared_ptr<rclcpp::Node> node,
                    std::string rosTopic,
                    ConvertFunc converter,
                    size_t qosHistoryDepth,
                    std::string cameraParamUri = "",
                    std::string cameraName = "",
                    bool lazyPublisher = true);

    /**
     * @brief Constructor for BridgePublisher with custom camera info data.
     *
     * @param daiMessageQueue Shared pointer to the DepthAI message queue.
     * @param node Shared pointer to the ROS 2 node.
     * @param rosTopic The ROS topic to publish messages to.
     * @param converter Function to convert DepthAI messages to ROS messages.
     * @param qosHistoryDepth History depth for the QoS settings.
     * @param cameraInfoData Camera info data for the ROS messages.
     * @param cameraName Name of the camera.
     * @param lazyPublisher Flag to enable lazy publishing.
     */
    BridgePublisher(std::shared_ptr<dai::MessageQueue> daiMessageQueue,
                    std::shared_ptr<rclcpp::Node> node,
                    std::string rosTopic,
                    ConvertFunc converter,
                    size_t qosHistoryDepth,
                    ImageMsgs::CameraInfo cameraInfoData,
                    std::string cameraName,
                    bool lazyPublisher = true);

    /**
     * Tag Dispacher function to to overload the Publisher to ImageTransport Publisher
     */
    std::shared_ptr<image_transport::CameraPublisher> advertise(int queueSize, std::true_type);

    /**
     * Tag Dispacher function to to overload the Publisher to use Default ros::Publisher
     */
    typename rclcpp::Publisher<RosMsg>::SharedPtr advertise(int queueSize, std::false_type);

    BridgePublisher(const BridgePublisher& other);

    void addPublisherCallback();

    void publishHelper(std::shared_ptr<DaiMsg> inData);

    void startPublisherThread();

    ~BridgePublisher();

   private:
    /**
     * adding this callback will allow you to still be able to consume
     * the data for other processing using get() function .
     */
    void daiCallback(const std::string& name, std::shared_ptr<dai::ADatatype> data);

    static const std::string LOG_TAG;
    std::shared_ptr<dai::MessageQueue> daiMessageQueue;
    ConvertFunc converter;

    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoPublisher;

    image_transport::ImageTransport it;
    ImageMsgs::CameraInfo cameraInfoData;
    CustomPublisher rosPublisher;

    std::thread readingThread;
    std::string rosTopic, camInfoFrameId, cameraName, cameraParamUri;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camInfoManager;
    bool isCallbackAdded = false;
    bool isImageMessage = false;  // used to enable camera info manager
    bool lazyPublisher = true;
};

template <class RosMsg, class DaiMsg>
const std::string BridgePublisher<RosMsg, DaiMsg>::LOG_TAG = "BridgePublisher";

template <class RosMsg, class DaiMsg>
BridgePublisher<RosMsg, DaiMsg>::BridgePublisher(std::shared_ptr<dai::MessageQueue> daiMessageQueue,
                                                 std::shared_ptr<rclcpp::Node> node,
                                                 std::string rosTopic,
                                                 ConvertFunc converter,
                                                 rclcpp::QoS qosSetting,
                                                 bool lazyPublisher)
    : daiMessageQueue(daiMessageQueue), node(node), converter(converter), it(node), rosTopic(rosTopic), lazyPublisher(lazyPublisher) {
    rosPublisher = node->create_publisher<RosMsg>(rosTopic, qosSetting);
}

template <class RosMsg, class DaiMsg>
BridgePublisher<RosMsg, DaiMsg>::BridgePublisher(std::shared_ptr<dai::MessageQueue> daiMessageQueue,
                                                 std::shared_ptr<rclcpp::Node> node,
                                                 std::string rosTopic,
                                                 ConvertFunc converter,
                                                 size_t qosHistoryDepth,
                                                 std::string cameraParamUri,
                                                 std::string cameraName,
                                                 bool lazyPublisher)
    : daiMessageQueue(daiMessageQueue),
      node(node),
      converter(converter),
      it(node),
      rosTopic(rosTopic),
      cameraParamUri(cameraParamUri),
      cameraName(cameraName),
      lazyPublisher(lazyPublisher) {
    rosPublisher = advertise(qosHistoryDepth, std::is_same<RosMsg, ImageMsgs::Image>{});
}

template <class RosMsg, class DaiMsg>
BridgePublisher<RosMsg, DaiMsg>::BridgePublisher(std::shared_ptr<dai::MessageQueue> daiMessageQueue,
                                                 std::shared_ptr<rclcpp::Node> node,
                                                 std::string rosTopic,
                                                 ConvertFunc converter,
                                                 size_t qosHistoryDepth,
                                                 ImageMsgs::CameraInfo cameraInfoData,
                                                 std::string cameraName,
                                                 bool lazyPublisher)
    : daiMessageQueue(daiMessageQueue),
      node(node),
      converter(converter),
      it(node),
      rosTopic(rosTopic),
      cameraInfoData(cameraInfoData),
      cameraName(cameraName),
      lazyPublisher(lazyPublisher) {
    rosPublisher = advertise(qosHistoryDepth, std::is_same<RosMsg, ImageMsgs::Image>{});
}

template <class RosMsg, class DaiMsg>
typename rclcpp::Publisher<RosMsg>::SharedPtr BridgePublisher<RosMsg, DaiMsg>::advertise(int queueSize, std::false_type) {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = rclcpp::QosOverridingOptions();
    if(!cameraName.empty()) {
        isImageMessage = true;
        camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(node.get(), cameraName, cameraParamUri);
        if(cameraParamUri.empty()) {
            camInfoManager->setCameraInfo(cameraInfoData);
        }
        cameraInfoPublisher = node->create_publisher<ImageMsgs::CameraInfo>(cameraName + "/camera_info", queueSize, options);
    }
    return node->create_publisher<RosMsg>(rosTopic, queueSize, options);
}

template <class RosMsg, class DaiMsg>
std::shared_ptr<image_transport::CameraPublisher> BridgePublisher<RosMsg, DaiMsg>::advertise(int queueSize, std::true_type) {
    if(!cameraName.empty()) {
        isImageMessage = true;
        camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(node.get(), cameraName, cameraParamUri);
        if(cameraParamUri.empty()) {
            camInfoManager->setCameraInfo(cameraInfoData);
        }
        rclcpp::PublisherOptions options;
        options.qos_overriding_options = rclcpp::QosOverridingOptions();
        cameraInfoPublisher = node->create_publisher<ImageMsgs::CameraInfo>(cameraName + "/camera_info", queueSize, options);
    }
    return std::make_shared<image_transport::CameraPublisher>(it.advertiseCamera(rosTopic, queueSize));
}

template <class RosMsg, class DaiMsg>
void BridgePublisher<RosMsg, DaiMsg>::daiCallback(const std::string& name, std::shared_ptr<dai::ADatatype> data) {
    auto daiDataPtr = std::dynamic_pointer_cast<DaiMsg>(data);
    publishHelper(daiDataPtr);
}

template <class RosMsg, class DaiMsg>
void BridgePublisher<RosMsg, DaiMsg>::startPublisherThread() {
    if(isCallbackAdded) {
        throw std::runtime_error(
            "addPublisherCallback() function adds a callback to the"
            "depthai which handles the publishing so no need to start"
            "the thread using startPublisherThread() ");
    }

    readingThread = std::thread([&]() {
        while(rclcpp::ok()) {
            auto daiDataPtr = daiMessageQueue->tryGet<DaiMsg>();
            if(daiDataPtr == nullptr) {
                continue;
            }

            publishHelper(daiDataPtr);
        }
    });
}

template <class RosMsg, class DaiMsg>
void BridgePublisher<RosMsg, DaiMsg>::addPublisherCallback() {
    daiMessageQueue->addCallback(std::bind(&BridgePublisher<RosMsg, DaiMsg>::daiCallback, this, std::placeholders::_1, std::placeholders::_2));
    isCallbackAdded = true;
}

template <class RosMsg, class DaiMsg>
void BridgePublisher<RosMsg, DaiMsg>::publishHelper(std::shared_ptr<DaiMsg> inDataPtr) {
    std::deque<RosMsg> opMsgs;

    int mainSubCount = 0;
    int infoSubCount = 0;

    if(isImageMessage) {
        infoSubCount = node->count_subscribers(cameraName + "/camera_info");
    }
    mainSubCount = node->count_subscribers(rosTopic);

    if(!lazyPublisher || mainSubCount > 0 || infoSubCount > 0) {
        converter(inDataPtr, opMsgs);

        while(opMsgs.size()) {
            RosMsg currMsg = opMsgs.front();
            if(mainSubCount > 0) {
                if constexpr(std::is_same_v<RosMsg, ImageMsgs::Image>) {
                    auto localCameraInfo = camInfoManager->getCameraInfo();
                    localCameraInfo.header.stamp = currMsg.header.stamp;
                    localCameraInfo.header.frame_id = currMsg.header.frame_id;
                    std::dynamic_pointer_cast<image_transport::CameraPublisher>(rosPublisher)->publish(currMsg, localCameraInfo);
                } else {
                    rosPublisher->publish(currMsg);
                }
            }

            if(infoSubCount > 0 && (std::is_same_v<RosMsg, ImageMsgs::CompressedImage> || std::is_same_v<RosMsg, FFMPEGMsgs::FFMPEGPacket>)) {
                auto localCameraInfo = camInfoManager->getCameraInfo();
                localCameraInfo.header.stamp = currMsg.header.stamp;
                localCameraInfo.header.frame_id = currMsg.header.frame_id;
                cameraInfoPublisher->publish(localCameraInfo);
            }
            opMsgs.pop_front();
        }
    }
}

template <class RosMsg, class DaiMsg>
BridgePublisher<RosMsg, DaiMsg>::~BridgePublisher() {
    if(readingThread.joinable()) readingThread.join();
}

}  // namespace depthai_bridge
