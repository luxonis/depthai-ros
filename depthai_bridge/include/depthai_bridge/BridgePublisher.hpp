#pragma once

#include <thread>
#include <type_traits>
#include <typeinfo>

#include "depthai/depthai.hpp"
#include "sensor_msgs/Image.h"
// #include <depthai_ros_msgs/DetectionDaiArray.h>
// #include <vision_msgs/Detection2DArray.h>
#include "ros/ros.h"
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <thread>

namespace dai::rosBridge {

template <class RosMsg>
ros::Publisher advertise(ros::NodeHandle& nh, image_transport::ImageTransport& it, std::string rosTopic, int queueSize, std::false_type){
  return nh.advertise<RosMsg>(rosTopic, queueSize);
}

template <class RosMsg>
image_transport::Publisher advertise(ros::NodeHandle& nh, image_transport::ImageTransport& it, std::string rosTopic, int queueSize, std::true_type){
  return it.advertise(rosTopic, queueSize);
}

template <class RosMsg, class SimMsg> class BridgePublisher {
  // using CustomPublisher = typename std::conditional<std::is_same<RosMsg,
  // sensor_msgs::Image>::value, image_transport::CameraPublisher,
  // ros::Publisher>::type;
  using ConvertFunc = std::function<void(std::shared_ptr<SimMsg>, RosMsg &)>;
  using CustomPublisher =
      typename std::conditional<std::is_same<RosMsg, sensor_msgs::Image>::value,
                                image_transport::Publisher,
                                ros::Publisher>::type;

private:
  /**
   * adding this callback will allow you to still be able to consume
   * the data for other processing using get() function .
   */
  void daiCallback(std::string name, std::shared_ptr<ADatatype> data);

  std::shared_ptr<dai::DataOutputQueue> _daiMessageQueue;
  ConvertFunc _converter;

  ros::NodeHandle _nh;
  // ros::Publisher _rosPublisher;
  ros::Publisher _cameraInfoPublisher;
  image_transport::ImageTransport _it;
  // image_transport::Publisher _imagePublisher;
  CustomPublisher _rosPublisher;
  std::thread _readingThread;
  std::string _rosTopic, _camInfoFrameId;
  std::unique_ptr<camera_info_manager::CameraInfoManager> _camInfoManager;
  bool _isCallbackAdded = false;
  bool _isImageMessage = false; // used to enable camera info manager

public:
BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                  ros::NodeHandle nh, std::string rosTopic,
                  ConvertFunc converter, int queueSize,
                  std::string cameraParamUri = "", std::string cameraName = "");

  BridgePublisher(const BridgePublisher& other);

  void addPubisherCallback();

  void publishHelper(std::shared_ptr<SimMsg> inData);

  void startPublisherThread();
  ~BridgePublisher();

};


template <class RosMsg, class SimMsg>
BridgePublisher<RosMsg, SimMsg>::BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                  ros::NodeHandle nh, std::string rosTopic,
                  ConvertFunc converter, int queueSize,
                  std::string cameraParamUri, std::string cameraName)
      : _daiMessageQueue(daiMessageQueue), _nh(nh), _converter(converter),
        _it(_nh), _rosTopic(rosTopic) {

    if (!cameraParamUri.empty() && !cameraName.empty()) {
      _isImageMessage = true;
      _camInfoManager =
          std::make_unique<camera_info_manager::CameraInfoManager>(
              ros::NodeHandle{_nh, cameraName}, cameraName, cameraParamUri);
      // _rosPublisher = _it.advertise(rosTopic, queueSize);
      _cameraInfoPublisher = _nh.advertise<sensor_msgs::CameraInfo>(
          cameraName + "/camera_info", queueSize);
    } 
    _rosPublisher = advertise<RosMsg>(_nh, _it, rosTopic, queueSize, std::is_same<RosMsg, sensor_msgs::Image>{});
    
  }

template <class RosMsg, class SimMsg>
BridgePublisher<RosMsg, SimMsg>::BridgePublisher(const BridgePublisher& other){
  _daiMessageQueue = other._daiMessageQueue;
  _nh = other._nh;
  _converter = other._converter;
  _rosTopic = other._rosTopic;
  _it = other._it;
  _rosPublisher = CustomPublisher(other._rosPublisher);

  if(other._isImageMessage){
    _isImageMessage = true;
    _camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(std::move(other._camInfoManager));
    _cameraInfoPublisher = ros::Publisher(other._cameraInfoPublisher);
  }
}

template <class RosMsg, class SimMsg>
void BridgePublisher<RosMsg, SimMsg>::startPublisherThread() {
  if (_isCallbackAdded) {
    std::runtime_error(
        "addPubisherCallback() function adds a callback to the"
        "depthai which handles the publishing so no need to start"
        "the thread using startPublisherThread() ");
  }

  _readingThread = std::thread([&]() {
    while (ros::ok()) {
      // auto daiDataPtr = _daiMessageQueue->get<SimMsg>();
      auto daiDataPtr = _daiMessageQueue->tryGet<SimMsg>();

      if (daiDataPtr == nullptr) {
        //  std::cout << "No data found!!!" <<std::endl;
        continue;
      }
      publishHelper(daiDataPtr);
    }
  });
}

template <class RosMsg, class SimMsg>
void BridgePublisher<RosMsg, SimMsg>::daiCallback(
    std::string name, std::shared_ptr<ADatatype> data) {
  // std::cout << "In callback " << name << std::endl;
  auto daiDataPtr = std::dynamic_pointer_cast<SimMsg>(data);
  publishHelper(daiDataPtr);
}

template <class RosMsg, class SimMsg>
void BridgePublisher<RosMsg, SimMsg>::addPubisherCallback() {
  _daiMessageQueue->addCallback(
      std::bind(&BridgePublisher<RosMsg, SimMsg>::daiCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  _isCallbackAdded = true;
}

template <class RosMsg, class SimMsg>
void BridgePublisher<RosMsg, SimMsg>::publishHelper(
    std::shared_ptr<SimMsg> inDataPtr) {

  RosMsg opMsg;
  if (_camInfoFrameId.empty()) {
    _converter(inDataPtr, opMsg);
    _camInfoFrameId = opMsg.header.frame_id;
  }

  if (_rosPublisher.getNumSubscribers() > 0) {
    // std::cout << "before  " << opMsg.height << " " << opMsg.width << " " <<
    // opMsg.data.size() << std::endl;
    _converter(inDataPtr, opMsg);
    // std::cout << opMsg.height << " " << opMsg.width << " " <<
    // opMsg.data.size() << std::endl;
    _rosPublisher.publish(opMsg);

    if (_isImageMessage && _cameraInfoPublisher.getNumSubscribers() > 0) {
      auto localCameraInfo = _camInfoManager->getCameraInfo();
      localCameraInfo.header.seq = opMsg.header.seq;
      localCameraInfo.header.stamp = opMsg.header.stamp;
      localCameraInfo.header.frame_id = opMsg.header.frame_id;
      _cameraInfoPublisher.publish(localCameraInfo);
    }
  }

  if (_isImageMessage && _rosPublisher.getNumSubscribers() == 0 &&
      _cameraInfoPublisher.getNumSubscribers() > 0) {
    _converter(inDataPtr, opMsg);
    auto localCameraInfo = _camInfoManager->getCameraInfo();
    localCameraInfo.header.seq = opMsg.header.seq;
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

} // namespace dai::rosBridge
