#pragma once

#include <thread>

#include "depthai/depthai.hpp"
#include <thread>
#include <camera_info_manager/camera_info_manager.h>

#include "rclcpp/rclcpp.hpp"

namespace dai::rosBridge {

template <class RosMsg, class SimMsg> 
class BridgePublisher {
public:
  using ConvertFunc = std::function<void (std::shared_ptr<SimMsg> , RosMsg&)>;

  BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                  std::shared_ptr<rclcpp::Node> node, std::string rosTopic,
                  ConvertFunc converter, int queueSize, 
                  rclcpp::QoS qosSetting = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
                  std::string cameraParamUri = "", std::string cameraName = "");

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
  
  std::shared_ptr<dai::DataOutputQueue> _daiMessageQueue;
  ConvertFunc _converter;
  
  std::shared_ptr<rclcpp::Node> _node;
  rclcpp::Publisher<RosMsg>::SharedPtr _rosPublisher;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _cameraInfoPublisher;
  std::thread _readingThread;
  std::string _rosTopic;
  std::unique_ptr<camera_info_manager::CameraInfoManager> _camInfoManager;
  bool _isCallbackAdded = false;
  bool _isImageMessage = false; // used to enable camera info manager
  
};


template <class RosMsg, class SimMsg> 
BridgePublisher<RosMsg, SimMsg>::BridgePublisher(
    std::shared_ptr<dai::DataOutputQueue> daiMessageQueue, std::shared_ptr<rclcpp::Node> node,
    std::string rosTopic, ConvertFunc converter, int queueSize, rclcpp::QoS qosSetting, std::string cameraParamUri, std::string cameraName)
    : _daiMessageQueue(daiMessageQueue), _node(node), _converter(converter),
      _rosTopic(rosTopic){
  
  _rosPublisher = _node->create_publisher(rosTopic, qosSetting);

  if(!cameraParamUri.empty() && !cameraName.empty()){
    _isImageMessage = true;
    _camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(ros::NodeHandle{_nh, cameraName}, cameraName, cameraParamUri);
    _cameraInfoPublisher = _node->create_publisher(cameraName + "/camera_info", qosSetting);
  }
}

template <class RosMsg, class SimMsg> 
BridgePublisher<RosMsg, SimMsg>::BridgePublisher(const BridgePublisher& other){
  _daiMessageQueue = other._daiMessageQueue;
  _node = other._node;
  _converter = other._converter;
  _rosTopic = other._rosTopic;
  _rosPublisher = other._rosPublisher;

  if(other._isImageMessage){
    _isImageMessage = true;
    _camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(std::move(other._camInfoManager));
    _cameraInfoPublisher = other._cameraInfoPublisher;
  }

}


template <class RosMsg, class SimMsg> 
void BridgePublisher<RosMsg, SimMsg>::startPublisherThread(){
  if(_isCallbackAdded){
    std::runtime_error("addPubisherCallback() function adds a callback to the"
                       "depthai which handles the publishing so no need to start" 
                       "the thread using startPublisherThread() ");
  }

  _readingThread = std::thread([&](){
    while(ros::ok()){
      // auto daiDataPtr = _daiMessageQueue->get<SimMsg>();
      auto daiDataPtr = _daiMessageQueue->tryGet<SimMsg>();

      if(daiDataPtr == nullptr) {
      //  std::cout << "No data found!!!" <<std::endl;
        continue;
      }
       publishHelper(daiDataPtr);

    }
  });
}

template <class RosMsg, class SimMsg> 
void BridgePublisher<RosMsg, SimMsg>::daiCallback(std::string name, std::shared_ptr<ADatatype> data){
  // std::cout << "In callback " << name << std::endl;
  auto daiDataPtr = std::dynamic_pointer_cast<SimMsg>(data);
  publishHelper(daiDataPtr);
 
      
}

template <class RosMsg, class SimMsg> 
void BridgePublisher<RosMsg, SimMsg>::addPubisherCallback(){
  _daiMessageQueue->addCallback(std::bind(&BridgePublisher<RosMsg, SimMsg>::daiCallback, this, std::placeholders::_1, std::placeholders::_2));
  _isCallbackAdded = true;
}

template <class RosMsg, class SimMsg> 
void BridgePublisher<RosMsg, SimMsg>::publishHelper(std::shared_ptr<SimMsg> inDataPtr){

  RosMsg opMsg;
 if(_rosPublisher.getNumSubscribers() > 0){
    // std::cout << "before  " << opMsg.height << " " << opMsg.width << " " << opMsg.data.size() << std::endl;
    _converter(inDataPtr, opMsg);
      // std::cout << opMsg.height << " " << opMsg.width << " " << opMsg.data.size() << std::endl;

    _rosPublisher.publish(opMsg);
    
    if(_isImageMessage && _cameraInfoPublisher.getNumSubscribers() > 0){
      auto localCameraInfo = _camInfoManager->getCameraInfo();
      localCameraInfo.header.seq = opMsg.header.seq;
      localCameraInfo.header.stamp = opMsg.header.stamp;
      _cameraInfoPublisher.publish(localCameraInfo);  
    }
  }

  if(_isImageMessage && _rosPublisher.getNumSubscribers() == 0 && _cameraInfoPublisher.getNumSubscribers() > 0){  
    _converter(inDataPtr, opMsg);
    auto localCameraInfo = _camInfoManager->getCameraInfo();
    localCameraInfo.header.seq = opMsg.header.seq;
    localCameraInfo.header.stamp = opMsg.header.stamp;
    _cameraInfoPublisher.publish(localCameraInfo);  
  }
}


template <class RosMsg, class SimMsg> 
BridgePublisher<RosMsg, SimMsg>::~BridgePublisher(){
  _readingThread.join();
}

// TODO(sachin): alternative methods to publish would be using walltimer here 
// (so I need to create async spinner for that or multithreaded nodehandle??), 
// ANd what about the callbacks ?


} // namespace dai::rosBridge
