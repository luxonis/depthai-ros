#pragma once
#include <deque>
#include <thread>
#include <type_traits>
#include <typeinfo>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/device/DataQueue.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

namespace dai {

namespace ros {

namespace StdMsgs = std_msgs::msg;
namespace ImageMsgs = sensor_msgs::msg;
using ImagePtr = ImageMsgs::Image::SharedPtr;
namespace rosOrigin = ::rclcpp;

template <class RosMsg, class SimMsg>
class BridgePublisher {
   public:
    using ConvertFunc = std::function<void(std::shared_ptr<SimMsg>, std::deque<RosMsg>&)>;
    using CustomPublisher = typename std::conditional<std::is_same<RosMsg, ImageMsgs::Image>::value,
                                                      std::shared_ptr<image_transport::Publisher>,
                                                      typename rclcpp::Publisher<RosMsg>::SharedPtr>::type;

    BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                    std::shared_ptr<rclcpp::Node> node,
                    std::string rosTopic,
                    ConvertFunc converter,
                    rclcpp::QoS qosSetting = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                    std::shared_ptr<rclcpp::Node> node,
                    std::string rosTopic,
                    ConvertFunc converter,
                    size_t qosHistoryDepth,
                    std::string cameraParamUri = "",
                    std::string cameraName = "");

    BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                    std::shared_ptr<rclcpp::Node> node,
                    std::string rosTopic,
                    ConvertFunc converter,
                    size_t qosHistoryDepth,
                    ImageMsgs::CameraInfo cameraInfoData,
                    std::string cameraName);

    /**
     * Tag Dispacher function to to overload the Publisher to ImageTransport Publisher
     */
    std::shared_ptr<image_transport::Publisher> advertise(int queueSize, std::true_type);

    /**
     * Tag Dispacher function to to overload the Publisher to use Default ros::Publisher
     */
    typename rclcpp::Publisher<RosMsg>::SharedPtr advertise(int queueSize, std::false_type);

    BridgePublisher(const BridgePublisher& other);

    void addPublisherCallback();

    void publishHelper(std::shared_ptr<SimMsg> inData);

    void startPublisherThread();

    ~BridgePublisher();

   private:
    /**
     * adding this callback will allow you to still be able to consume
     * the data for other processing using get() function .
     */
    void daiCallback(std::string name, std::shared_ptr<ADatatype> data);

    static const std::string LOG_TAG;
    std::shared_ptr<dai::DataOutputQueue> _daiMessageQueue;
    ConvertFunc _converter;

    std::shared_ptr<rclcpp::Node> _node;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _cameraInfoPublisher;

    image_transport::ImageTransport _it;
    ImageMsgs::CameraInfo _cameraInfoData;
    CustomPublisher _rosPublisher;

    std::thread _readingThread;
    std::string _rosTopic, _camInfoFrameId, _cameraName, _cameraParamUri;
    std::unique_ptr<camera_info_manager::CameraInfoManager> _camInfoManager;
    bool _isCallbackAdded = false;
    bool _isImageMessage = false;  // used to enable camera info manager
};

template <class RosMsg, class SimMsg>
const std::string BridgePublisher<RosMsg, SimMsg>::LOG_TAG = "BridgePublisher";

template <class RosMsg, class SimMsg>
BridgePublisher<RosMsg, SimMsg>::BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                                                 std::shared_ptr<rclcpp::Node> node,
                                                 std::string rosTopic,
                                                 ConvertFunc converter,
                                                 rclcpp::QoS qosSetting)
    : _daiMessageQueue(daiMessageQueue), _node(node), _converter(converter), _it(node), _rosTopic(rosTopic) {
    _rosPublisher = _node->create_publisher<RosMsg>(_rosTopic, qosSetting);
}

template <class RosMsg, class SimMsg>
BridgePublisher<RosMsg, SimMsg>::BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                                                 std::shared_ptr<rclcpp::Node> node,
                                                 std::string rosTopic,
                                                 ConvertFunc converter,
                                                 size_t qosHistoryDepth,
                                                 std::string cameraParamUri,
                                                 std::string cameraName)
    : _daiMessageQueue(daiMessageQueue),
      _node(node),
      _converter(converter),
      _it(node),
      _rosTopic(rosTopic),
      _cameraParamUri(cameraParamUri),
      _cameraName(cameraName) {
    _rosPublisher = advertise(qosHistoryDepth, std::is_same<RosMsg, ImageMsgs::Image>{});
}

template <class RosMsg, class SimMsg>
BridgePublisher<RosMsg, SimMsg>::BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                                                 std::shared_ptr<rclcpp::Node> node,
                                                 std::string rosTopic,
                                                 ConvertFunc converter,
                                                 size_t qosHistoryDepth,
                                                 ImageMsgs::CameraInfo cameraInfoData,
                                                 std::string cameraName)
    : _daiMessageQueue(daiMessageQueue),
      _node(node),
      _converter(converter),
      _it(node),
      _rosTopic(rosTopic),
      _cameraInfoData(cameraInfoData),
      _cameraName(cameraName) {
    _rosPublisher = advertise(qosHistoryDepth, std::is_same<RosMsg, ImageMsgs::Image>{});
}

template <class RosMsg, class SimMsg>
typename rclcpp::Publisher<RosMsg>::SharedPtr BridgePublisher<RosMsg, SimMsg>::advertise(int queueSize, std::false_type) {
    rclcpp::PublisherOptions options;
    options.qos_overriding_options = rclcpp::QosOverridingOptions();
    return _node->create_publisher<RosMsg>(_rosTopic, queueSize, options);
}

template <class RosMsg, class SimMsg>
std::shared_ptr<image_transport::Publisher> BridgePublisher<RosMsg, SimMsg>::advertise(int queueSize, std::true_type) {
    if(!_cameraName.empty()) {
        _isImageMessage = true;
        _camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(_node.get(), _cameraName, _cameraParamUri);
        if(_cameraParamUri.empty()) {
            _camInfoManager->setCameraInfo(_cameraInfoData);
        }
        rclcpp::PublisherOptions options;
        options.qos_overriding_options = rclcpp::QosOverridingOptions();
        _cameraInfoPublisher = _node->create_publisher<ImageMsgs::CameraInfo>(_cameraName + "/camera_info", queueSize, options);
    }
    return std::make_shared<image_transport::Publisher>(_it.advertise(_rosTopic, queueSize));
}

template <class RosMsg, class SimMsg>
void BridgePublisher<RosMsg, SimMsg>::daiCallback(std::string name, std::shared_ptr<ADatatype> data) {
    // std::cout << "In callback " << name << std::endl;
    auto daiDataPtr = std::dynamic_pointer_cast<SimMsg>(data);
    publishHelper(daiDataPtr);
}

template <class RosMsg, class SimMsg>
void BridgePublisher<RosMsg, SimMsg>::startPublisherThread() {
    if(_isCallbackAdded) {
        std::runtime_error(
            "addPublisherCallback() function adds a callback to the"
            "depthai which handles the publishing so no need to start"
            "the thread using startPublisherThread() ");
    }

    _readingThread = std::thread([&]() {
        int messageCounter = 0;
        while(rosOrigin::ok()) {
            // auto daiDataPtr = _daiMessageQueue->get<SimMsg>();
            auto daiDataPtr = _daiMessageQueue->tryGet<SimMsg>();
            if(daiDataPtr == nullptr) {
                messageCounter++;
                if(messageCounter > 2000000) {
                    messageCounter = 0;
                }
                continue;
            }

            if(messageCounter != 0) {
                messageCounter = 0;
            }
            publishHelper(daiDataPtr);
        }
    });
}

template <class RosMsg, class SimMsg>
void BridgePublisher<RosMsg, SimMsg>::addPublisherCallback() {
    _daiMessageQueue->addCallback(std::bind(&BridgePublisher<RosMsg, SimMsg>::daiCallback, this, std::placeholders::_1, std::placeholders::_2));
    _isCallbackAdded = true;
}

template <class RosMsg, class SimMsg>
void BridgePublisher<RosMsg, SimMsg>::publishHelper(std::shared_ptr<SimMsg> inDataPtr) {
    std::deque<RosMsg> opMsgs;

    int infoSubCount = 0, mainSubCount = 0;

    if(_isImageMessage) {
        infoSubCount = _node->count_subscribers(_cameraName + "/camera_info");
    }
    mainSubCount = _node->count_subscribers(_rosTopic);

    if(mainSubCount > 0 || infoSubCount > 0) {
        _converter(inDataPtr, opMsgs);

        while(opMsgs.size()) {
            RosMsg currMsg = opMsgs.front();
            if(mainSubCount > 0) {
                _rosPublisher->publish(currMsg);
            }

            if(infoSubCount > 0) {
                auto localCameraInfo = _camInfoManager->getCameraInfo();
                localCameraInfo.header.stamp = currMsg.header.stamp;
                localCameraInfo.header.frame_id = currMsg.header.frame_id;
                _cameraInfoPublisher->publish(localCameraInfo);
            }
            opMsgs.pop_front();
        }
    }
}

template <class RosMsg, class SimMsg>
BridgePublisher<RosMsg, SimMsg>::~BridgePublisher() {
    if(_readingThread.joinable()) _readingThread.join();
}

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
