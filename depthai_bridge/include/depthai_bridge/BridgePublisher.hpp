#pragma once

#include <thread>
#include <type_traits>
#include <typeinfo>

#include "depthai/depthai.hpp"
// #include <depthai_ros_msgs/DetectionDaiArray.h>
// #include <vision_msgs/Detection2DArray.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

#ifdef IS_ROS2
    #include "rclcpp/rclcpp.hpp"
    #include "sensor_msgs/msg/camera_info.hpp"
    #include "sensor_msgs/msg/image.hpp"
    #include "std_msgs/msg/header.hpp"
    namespace StdMsgs = std_msgs::msg;
    namespace ImageMsgs = sensor_msgs::msg;
    using ImagePtr = ImageMsgs::Image::SharedPtr;
#else
    #include <ros/console.h>
    #include <ros/ros.h>
    #include <boost/make_shared.hpp>
    #include <boost/shared_ptr.hpp>
    #include "sensor_msgs/Image.h"
    namespace StdMsgs = std_msgs;
    namespace ImageMsgs = sensor_msgs;
    using ImagePtr = ImageMsgs::ImagePtr;
#endif

#include <thread>


namespace dai::rosBridge {

template <class RosMsg, class SimMsg>
class BridgePublisher {
   public:
    using ConvertFunc = std::function<void(std::shared_ptr<SimMsg>, RosMsg&)>;
    
    #ifdef IS_ROS2
        using CustomPublisher = typename std::conditional<std::is_same<RosMsg, ImageMsgs::Image>::value, std::shared_ptr<image_transport::Publisher>, rclcpp::Publisher<RosMsg>::SharedPtr::type;

      BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                  std::shared_ptr<rclcpp::Node> node,
                  std::string rosTopic,
                  ConvertFunc converter,
                  rmw_qos_profile_t qos_profile = rmw_qos_profile_default);
 
      // BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
      //             std::shared_ptr<rclcpp::Node> node,
      //             std::string rosTopic,
      //             ConvertFunc converter,
      //             rmw_qos_profile_t qos_profile = rmw_qos_profile_default);

      BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                  std::shared_ptr<rclcpp::Node> node,
                  std::string rosTopic,
                  ConvertFunc converter,
                  size_t 	qosHistoryDepth,
                  std::string cameraParamUri = "",
                  std::string cameraName = "");
 
      BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                  std::shared_ptr<rclcpp::Node> node,
                  std::string rosTopic,
                  ConvertFunc converter,
                  size_t 	qosHistoryDepth,
                  ImageMsgs::CameraInfo cameraInfo,
                  std::string cameraName);
    #else
        using CustomPublisher = typename std::conditional<std::is_same<RosMsg, ImageMsgs::Image>::value, std::shared_ptr<image_transport::Publisher>, std::shared_ptr<ros::Publisher>>::type;
    
        BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                        ros::NodeHandle nh,
                        std::string rosTopic,
                        ConvertFunc converter,
                        int queueSize,
                        std::string cameraParamUri = "",
                        std::string cameraName = "");

        BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                        ros::NodeHandle nh,
                        std::string rosTopic,
                        ConvertFunc converter,
                        int queueSize,
                        ImageMsgs::CameraInfo cameraInfo,
                        std::string cameraName);
    #endif


    BridgePublisher(const BridgePublisher& other);

    void addPubisherCallback();

    void publishHelper(std::shared_ptr<SimMsg> inData);

    void startPublisherThread();

    ~BridgePublisher();

   private:
    /**
     * adding this callback will allow you to still be able to consume
     * the data for other processing using get() function .
     */
    void daiCallback(std::string name, std::shared_ptr<ADatatype> data);

    /**
     * Tag Dispacher function to to overload the Publisher to ImageTransport Publisher
     */
    image_transport::Publisher advertise(int queueSize, std::true_type);

    /**
     * Tag Dispacher function to to overload the Publisher to use Default ros::Publisher
     */
    ros::Publisher advertise(int queueSize, std::false_type);

    static const std::string LOG_TAG;
    std::shared_ptr<dai::DataOutputQueue> _daiMessageQueue;
    ConvertFunc _converter;

    #ifdef IS_ROS2
      std::shared_ptr<rclcpp::Node> _node;
      rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _cameraInfoPublisher;
    #else
      ros::NodeHandle _nh;
      std::shared_ptr<ros::Publisher> _cameraInfoPublisher;
    #endif

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

#ifdef IS_ROS2
    template <class RosMsg, class SimMsg> 
    BridgePublisher<RosMsg, SimMsg>::BridgePublisher(
        std::shared_ptr<dai::DataOutputQueue> daiMessageQueue, std::shared_ptr<rclcpp::Node> node,
        std::string rosTopic, ConvertFunc converter, rmw_qos_profile_t qos_profile)
        : _daiMessageQueue(daiMessageQueue), _node(node), _converter(converter),
        _rosTopic(rosTopic){

        _rosPublisher = _node->create_publisher<RosMsg>(_rosTopic, qos_profile);
        // _rosPublisher = _node->create_publisher<RosMsg>(_rosTopic, qosSetting);
        // if(!cameraParamUri.empty() && !cameraName.empty()){
        //     _isImageMessage = true;
        //     _camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(_node.get(), cameraName, cameraParamUri);
        //     _cameraInfoTopic = cameraName + "/camera_info";
        //     _cameraInfoPublisher = _node->create_publisher<ImageMsgs::msg::CameraInfo>(_cameraInfoTopic, qosSetting);
        // }
    }

    BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                std::shared_ptr<rclcpp::Node> node,
                std::string rosTopic,
                ConvertFunc converter,
                size_t qosHistoryDepth,
                std::string cameraParamUri,
                std::string cameraName)
                : _daiMessageQueue(daiMessageQueue), _node(node), _converter(converter), _it(node),
      _rosTopic(rosTopic), _cameraParamUri(cameraParamUri), _cameraName(cameraName){
      _rosPublisher = advertise(qosHistoryDepth, std::is_same<RosMsg, ImageMsgs::Image>{});
    }

    BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                std::shared_ptr<rclcpp::Node> node,
                std::string rosTopic,
                ConvertFunc converter,
                size_t qosHistoryDepth,
                ImageMsgs::CameraInfo cameraInfo,
                std::string cameraName)
                : _daiMessageQueue(daiMessageQueue), _node(node), _converter(converter), _it(node),
      _rosTopic(rosTopic), _cameraInfoData(cameraInfoData), _cameraName(cameraName){
      _rosPublisher = advertise(qosHistoryDepth, std::is_same<RosMsg, ImageMsgs::Image>{});
    }

    template <class RosMsg, class SimMsg>
    rclcpp::Publisher<RosMsg>::SharedPtr  BridgePublisher<RosMsg, SimMsg>::advertise(int queueSize, std::false_type) {
        return _node->create_publisher<RosMsg>(_rosTopic, queueSize);
    }

    template <class RosMsg, class SimMsg>
    std::shared_ptr<image_transport::Publisher> BridgePublisher<RosMsg, SimMsg>::advertise(int queueSize, std::true_type) {
        if(!_cameraName.empty()) {
            _isImageMessage = true;
            _camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(_node.get(), _cameraName, _cameraParamUri);
            if(_cameraParamUri.empty()) {
            _camInfoManager->setCameraInfo(_cameraInfoData);
            }
            // _rosPublisher = _it.advertise(rosTopic, queueSize);
            _cameraInfoPublisher = _node->create_publisher<ImageMsgs::CameraInfo>(_cameraName + "/camera_info", queueSize);
        }
        return std::make_shared<image_transport::Publisher>(_it.advertise(_rosTopic, queueSize));
    }
#else
    template <class RosMsg, class SimMsg>
    BridgePublisher<RosMsg, SimMsg>::BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                                                    ros::NodeHandle nh,
                                                    std::string rosTopic,
                                                    ConvertFunc converter,
                                                    int queueSize,
                                                    std::string cameraParamUri,
                                                    std::string cameraName)
        : _daiMessageQueue(daiMessageQueue),
        _nh(nh),
        _converter(converter),
        _it(_nh),
        _rosTopic(rosTopic),
        _cameraParamUri(cameraParamUri),
        _cameraName(cameraName) {
        ROS_DEBUG_STREAM_NAMED(LOG_TAG, "Publisher Type : " << typeid(CustomPublisher).name());
        _rosPublisher = advertise(queueSize, std::is_same<RosMsg, ImageMsgs::Image>{});
    }

    template <class RosMsg, class SimMsg>
    BridgePublisher<RosMsg, SimMsg>::BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                                                    ros::NodeHandle nh,
                                                    std::string rosTopic,
                                                    ConvertFunc converter,
                                                    int queueSize,
                                                    ImageMsgs::CameraInfo cameraInfoData,
                                                    std::string cameraName)
        : _daiMessageQueue(daiMessageQueue),
        _nh(nh),
        _converter(converter),
        _it(_nh),
        _rosTopic(rosTopic),
        _cameraInfoData(cameraInfoData),
        _cameraName(cameraName) {
        ROS_DEBUG_STREAM_NAMED(LOG_TAG, "Publisher Type : " << typeid(CustomPublisher).name());
        _rosPublisher = advertise(queueSize, std::is_same<RosMsg, ImageMsgs::Image>{});
    }

    template <class RosMsg, class SimMsg>
    void BridgePublisher<RosMsg, SimMsg>::daiCallback(std::string name, std::shared_ptr<ADatatype> data) {
        // std::cout << "In callback " << name << std::endl;
        auto daiDataPtr = std::dynamic_pointer_cast<SimMsg>(data);
        publishHelper(daiDataPtr);
    }

    template <class RosMsg, class SimMsg>
    std::shared_ptr<ros::Publisher> BridgePublisher<RosMsg, SimMsg>::advertise(int queueSize, std::false_type) {
        return std::make_shared<ros::Publisher>(_nh.advertise<RosMsg>(_rosTopic, queueSize));
    }

    template <class RosMsg, class SimMsg>
    std::shared_ptr<image_transport::Publisher> BridgePublisher<RosMsg, SimMsg>::advertise(int queueSize, std::true_type) {
        if(!_cameraName.empty()) {
            _isImageMessage = true;
            _camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(ros::NodeHandle{_nh, _cameraName}, _cameraName, _cameraParamUri);
            if(_cameraParamUri.empty()) {
                _camInfoManager->setCameraInfo(_cameraInfoData);
            }
            _cameraInfoPublisher = _nh.advertise<ImageMsgs::CameraInfo>(_cameraName + "/camera_info", queueSize);
        }
        return std::make_shared<image_transport::Publisher>(_it.advertise(_rosTopic, queueSize));
    }

    template <class RosMsg, class SimMsg>
    BridgePublisher<RosMsg, SimMsg>::BridgePublisher(const BridgePublisher& other) {
        _daiMessageQueue = other._daiMessageQueue;
        _nh = other._nh;
        _converter = other._converter;
        _rosTopic = other._rosTopic;
        _it = other._it;
        _rosPublisher = CustomPublisher(other._rosPublisher);

        if(other._isImageMessage) {
            _isImageMessage = true;
            _camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(std::move(other._camInfoManager));
            _cameraInfoPublisher = ros::Publisher(other._cameraInfoPublisher);
        }
    }
#endif






template <class RosMsg, class SimMsg>
void BridgePublisher<RosMsg, SimMsg>::startPublisherThread() {
    if(_isCallbackAdded) {
        std::runtime_error(
            "addPubisherCallback() function adds a callback to the"
            "depthai which handles the publishing so no need to start"
            "the thread using startPublisherThread() ");
    }

    _readingThread = std::thread([&]() {
        int messageCounter = 0;
        while(ros::ok()) {
            // auto daiDataPtr = _daiMessageQueue->get<SimMsg>();
            auto daiDataPtr = _daiMessageQueue->tryGet<SimMsg>();
            if(daiDataPtr == nullptr) {
                messageCounter++;
                if(messageCounter > 2000000) {
                    ROS_WARN_STREAM_NAMED(LOG_TAG,
                                          "Missing " << messageCounter << " number of Incoming message from Depthai Queue " << _daiMessageQueue->getName());
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
void BridgePublisher<RosMsg, SimMsg>::addPubisherCallback() {
    _daiMessageQueue->addCallback(std::bind(&BridgePublisher<RosMsg, SimMsg>::daiCallback, this, std::placeholders::_1, std::placeholders::_2));
    _isCallbackAdded = true;
}

template <class RosMsg, class SimMsg>
void BridgePublisher<RosMsg, SimMsg>::publishHelper(std::shared_ptr<SimMsg> inDataPtr) {
    RosMsg opMsg;
    if(_camInfoFrameId.empty()) {
        _converter(inDataPtr, opMsg);
        _camInfoFrameId = opMsg.header.frame_id;
    }

    ROS_DEBUG_STREAM_NAMED(LOG_TAG, "Total number of subscribers to " << _rosTopic << " is :" << _rosPublisher.getNumSubscribers());
    if(_isImageMessage) {
        ROS_DEBUG_STREAM_NAMED(LOG_TAG,
                               "Total number of subscribers to " << _cameraInfoPublisher.getTopic() << " is :" << _cameraInfoPublisher.getNumSubscribers());
    }

    if(_rosPublisher.getNumSubscribers() > 0) {
        _converter(inDataPtr, opMsg);
        // std::cout << opMsg.height << " " << opMsg.width << " " <<
        // opMsg.data.size() << std::endl;
        _rosPublisher->publish(opMsg);

        if(_isImageMessage && _cameraInfoPublisher.getNumSubscribers() > 0) {
            auto localCameraInfo = _camInfoManager->getCameraInfo();
            #ifndef IS_ROS2
              localCameraInfo.header.seq = opMsg.header.seq;
            #endif
            localCameraInfo.header.stamp = opMsg.header.stamp;
            localCameraInfo.header.frame_id = opMsg.header.frame_id;
            _cameraInfoPublisher->publish(localCameraInfo);
        }
    }

    if(_isImageMessage && _rosPublisher.getNumSubscribers() == 0 && _cameraInfoPublisher.getNumSubscribers() > 0) {
        _converter(inDataPtr, opMsg);
        auto localCameraInfo = _camInfoManager->getCameraInfo();
        #ifndef IS_ROS2
          localCameraInfo.header.seq = opMsg.header.seq;
        #endif
        localCameraInfo.header.stamp = opMsg.header.stamp;
        localCameraInfo.header.frame_id = _camInfoFrameId;
        _cameraInfoPublisher.publish(localCameraInfo);
    }
}

template <class RosMsg, class SimMsg>
BridgePublisher<RosMsg, SimMsg>::~BridgePublisher() {
    _readingThread.join();
}

// TODO(sachin): alternative methods to publish would be using walltimer here
// (so I need to create async spinner for that or multithreaded nodehandle??),
// ANd what about the callbacks ?

}  // namespace dai::rosBridge
