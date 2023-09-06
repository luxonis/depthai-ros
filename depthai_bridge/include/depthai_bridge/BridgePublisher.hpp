#pragma once

#include <deque>
#include <thread>
#include <type_traits>
#include <typeinfo>

#include "boost/make_shared.hpp"
#include "boost/shared_ptr.hpp"
#include "camera_info_manager/camera_info_manager.h"
#include "depthai/depthai.hpp"
#include "image_transport/image_transport.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

namespace dai {

namespace ros {

namespace StdMsgs = std_msgs;
namespace ImageMsgs = sensor_msgs;
using ImagePtr = ImageMsgs::ImagePtr;
namespace rosOrigin = ::ros;

template <class RosMsg, class SimMsg>
class BridgePublisher {
   public:
    using ConvertFunc = std::function<void(std::shared_ptr<SimMsg>, std::deque<RosMsg>&)>;

    using CustomPublisher = typename std::
        conditional<std::is_same<RosMsg, ImageMsgs::Image>::value, std::shared_ptr<image_transport::Publisher>, std::shared_ptr<rosOrigin::Publisher> >::type;

    BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                    rosOrigin::NodeHandle nh,
                    std::string rosTopic,
                    ConvertFunc converter,
                    int queueSize,
                    std::string cameraParamUri = "",
                    std::string cameraName = "",
                    bool lazyPublisher = true);

    BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                    rosOrigin::NodeHandle nh,
                    std::string rosTopic,
                    ConvertFunc converter,
                    int queueSize,
                    ImageMsgs::CameraInfo cameraInfoData,
                    std::string cameraName,
                    bool lazyPublisher = true);

    /**
     * Tag Dispacher function to to overload the Publisher to ImageTransport Publisher
     */
    std::shared_ptr<image_transport::Publisher> advertise(int queueSize, std::true_type);

    /**
     * Tag Dispacher function to to overload the Publisher to use Default ros::Publisher
     */
    std::shared_ptr<rosOrigin::Publisher> advertise(int queueSize, std::false_type);

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

    rosOrigin::NodeHandle _nh;
    std::shared_ptr<rosOrigin::Publisher> _cameraInfoPublisher;

    image_transport::ImageTransport _it;
    ImageMsgs::CameraInfo _cameraInfoData;
    CustomPublisher _rosPublisher;

    std::thread _readingThread;
    std::string _rosTopic, _camInfoFrameId, _cameraName, _cameraParamUri;
    std::unique_ptr<camera_info_manager::CameraInfoManager> _camInfoManager;
    bool _isCallbackAdded = false;
    bool _isImageMessage = false;  // used to enable camera info manager
    bool _lazyPublisher = true;
};

template <class RosMsg, class SimMsg>
const std::string BridgePublisher<RosMsg, SimMsg>::LOG_TAG = "BridgePublisher";

template <class RosMsg, class SimMsg>
BridgePublisher<RosMsg, SimMsg>::BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                                                 rosOrigin::NodeHandle nh,
                                                 std::string rosTopic,
                                                 ConvertFunc converter,
                                                 int queueSize,
                                                 std::string cameraParamUri,
                                                 std::string cameraName,
                                                 bool lazyPublisher)
    : _daiMessageQueue(daiMessageQueue),
      _converter(converter),
      _nh(nh),
      _it(_nh),
      _rosTopic(rosTopic),
      _cameraParamUri(cameraParamUri),
      _cameraName(cameraName),
      _lazyPublisher(lazyPublisher) {
    // ROS_DEBUG_STREAM_NAMED(LOG_TAG, "Publisher Type : " << typeid(CustomPublisher).name());
    _rosPublisher = advertise(queueSize, std::is_same<RosMsg, ImageMsgs::Image>{});
}

template <class RosMsg, class SimMsg>
BridgePublisher<RosMsg, SimMsg>::BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                                                 rosOrigin::NodeHandle nh,
                                                 std::string rosTopic,
                                                 ConvertFunc converter,
                                                 int queueSize,
                                                 ImageMsgs::CameraInfo cameraInfoData,
                                                 std::string cameraName,
                                                 bool lazyPublisher)
    : _daiMessageQueue(daiMessageQueue),
      _nh(nh),
      _converter(converter),
      _it(_nh),
      _cameraInfoData(cameraInfoData),
      _rosTopic(rosTopic),
      _cameraName(cameraName),
      _lazyPublisher(lazyPublisher) {
    // ROS_DEBUG_STREAM_NAMED(LOG_TAG, "Publisher Type : " << typeid(CustomPublisher).name());
    _rosPublisher = advertise(queueSize, std::is_same<RosMsg, ImageMsgs::Image>{});
}

template <class RosMsg, class SimMsg>
std::shared_ptr<rosOrigin::Publisher> BridgePublisher<RosMsg, SimMsg>::advertise(int queueSize, std::false_type) {
    return std::make_shared<rosOrigin::Publisher>(_nh.advertise<RosMsg>(_rosTopic, queueSize));
}

template <class RosMsg, class SimMsg>
std::shared_ptr<image_transport::Publisher> BridgePublisher<RosMsg, SimMsg>::advertise(int queueSize, std::true_type) {
    if(!_cameraName.empty()) {
        _isImageMessage = true;
        _camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(rosOrigin::NodeHandle{_nh, _cameraName}, _cameraName, _cameraParamUri);
        if(_cameraParamUri.empty()) {
            _camInfoManager->setCameraInfo(_cameraInfoData);
        }
        _cameraInfoPublisher = std::make_shared<rosOrigin::Publisher>(_nh.advertise<ImageMsgs::CameraInfo>(_cameraName + "/camera_info", queueSize));
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
        _cameraInfoPublisher = rosOrigin::Publisher(other._cameraInfoPublisher);
    }
}

template <class RosMsg, class SimMsg>
void BridgePublisher<RosMsg, SimMsg>::daiCallback(std::string name, std::shared_ptr<ADatatype> data) {
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
        infoSubCount = _cameraInfoPublisher->getNumSubscribers();
    }
    mainSubCount = _rosPublisher->getNumSubscribers();

    if(!_lazyPublisher || (mainSubCount > 0 || infoSubCount > 0)) {
        _converter(inDataPtr, opMsgs);

        while(opMsgs.size()) {
            RosMsg currMsg = opMsgs.front();
            if(mainSubCount > 0) {
                _rosPublisher->publish(currMsg);
            }

            if(infoSubCount > 0) {
                // if (_isImageMessage){
                //     _camInfoFrameId = curr.header.frame_id
                // }
                auto localCameraInfo = _camInfoManager->getCameraInfo();
                localCameraInfo.header.seq = currMsg.header.seq;
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
