#pragma once

#include <thread>

#include "depthai/depthai.hpp"
#include "sensor_msgs/Image.h"
#include <depthai_ros_msgs/DetectionDaiArray.h>
#include <vision_msgs/Detection2DArray.h>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <thread>
#include <camera_info_manager/camera_info_manager.h>

#include "ros/ros.h"

namespace dai::rosBridge {

template <class RosMsg, class SimMsg> 
class BridgePublisher {
public:
  using ConvertFunc = std::function<void (std::shared_ptr<SimMsg> , RosMsg&)>;

  BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                  ros::NodeHandle &nh, std::string rosTopic,
                  ConvertFunc converter, int queueSize, std::string cameraParamUri = "", std::string cameraName = "");

  BridgePublisher(const BridgePublisher& other);

  void addPubisherCallback();

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
  
  ros::NodeHandle _nh;
  ros::Publisher _rosPublisher;
  ros::Publisher _cameraInfoPublisher;
  std::thread _readingThread;
  std::string _rosTopic;
  std::unique_ptr<camera_info_manager::CameraInfoManager> _camInfoManager;
  bool _isCallbackAdded = false;
  bool _isImageMessage = false; // used to enable camera info manager
  
};


template <class RosMsg, class SimMsg> 
BridgePublisher<RosMsg, SimMsg>::BridgePublisher(
    std::shared_ptr<dai::DataOutputQueue> daiMessageQueue, ros::NodeHandle &nh,
    std::string rosTopic, ConvertFunc converter, int queueSize, std::string cameraParamUri, std::string cameraName)
    : _daiMessageQueue(daiMessageQueue), _nh(nh), _converter(converter),
      _rosTopic(rosTopic){
  
  _rosPublisher = _nh.advertise<RosMsg>(rosTopic, queueSize);

  if(!cameraParamUri.empty() && !cameraName.empty()){
    _isImageMessage = true;
    _camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(ros::NodeHandle{_nh, cameraName}, cameraName, cameraParamUri);
    _cameraInfoPublisher = _nh.advertise<sensor_msgs::CameraInfo>(cameraName + "/camera_info", queueSize);
  }
}

template <class RosMsg, class SimMsg> 
BridgePublisher<RosMsg, SimMsg>::BridgePublisher(const BridgePublisher& other){
  _daiMessageQueue = other._daiMessageQueue;
  _nh = other._nh;
  _converter = other._converter;
  _rosTopic = other._rosTopic;
  _rosPublisher = ros::Publisher(other._rosPublisher);

  if(other._isImageMessage){
    _isImageMessage = true;
    _camInfoManager = std::make_unique<camera_info_manager::CameraInfoManager>(std::move(other._camInfoManager));
    _cameraInfoPublisher = ros::Publisher(other._cameraInfoPublisher);
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
      RosMsg opMsg;
      if(_rosPublisher.getNumSubscribers() > 0){        
        _converter(daiDataPtr, opMsg);
        _rosPublisher.publish(opMsg);
        
        if(_isImageMessage && _cameraInfoPublisher.getNumSubscribers() > 0){
          auto localCameraInfo = _camInfoManager->getCameraInfo();
          localCameraInfo.header.stamp = opMsg.header.stamp;
          _cameraInfoPublisher.publish(localCameraInfo);  
        }
      }
    
      if(_rosPublisher.getNumSubscribers() == 0 && _cameraInfoPublisher.getNumSubscribers() > 0)
        ROS_WARN("cameraInfo publishes only after subscribing to image topic also" );
        
    }
  });
}

template <class RosMsg, class SimMsg> 
void BridgePublisher<RosMsg, SimMsg>::daiCallback(std::string name, std::shared_ptr<ADatatype> data){
  // std::cout << "In callback " << name << std::endl;
  auto daiDataPtr = std::dynamic_pointer_cast<SimMsg>(data);
  // if(daiDataPtr == nullptr) {
  //      std::cout << "No data found!!!" <<std::endl;
  //       // continue;
  //     }
  //     else{
  //       std::cout << "Heh   data found..............................................!!!" << std::endl;
      
  //     }  
  
  RosMsg opMsg;
  if(_rosPublisher.getNumSubscribers() > 0){
    // std::cout << "before  " << opMsg.height << " " << opMsg.width << " " << opMsg.data.size() << std::endl;
    _converter(daiDataPtr, opMsg);
      // std::cout << opMsg.height << " " << opMsg.width << " " << opMsg.data.size() << std::endl;

    _rosPublisher.publish(opMsg);
    
    if(_isImageMessage && _cameraInfoPublisher.getNumSubscribers() > 0){
      auto localCameraInfo = _camInfoManager->getCameraInfo();
      // std::cout << opMsg.header.stamp << "stamp " << std::endl;
      localCameraInfo.header.stamp = opMsg.header.stamp;
      _cameraInfoPublisher.publish(localCameraInfo);  
    }
  }

  if(_rosPublisher.getNumSubscribers() == 0 && _cameraInfoPublisher.getNumSubscribers() > 0)
    ROS_WARN("cameraInfo publishes only after subscribing to image topic also");
      
}

template <class RosMsg, class SimMsg> 
void BridgePublisher<RosMsg, SimMsg>::addPubisherCallback(){
  _daiMessageQueue->addCallback(std::bind(&BridgePublisher<RosMsg, SimMsg>::daiCallback, this, std::placeholders::_1, std::placeholders::_2));
  _isCallbackAdded = true;
}


template <class RosMsg, class SimMsg> 
BridgePublisher<RosMsg, SimMsg>::~BridgePublisher(){
  _readingThread.join();
}

// TODO(sachin): alternative methods to publish would be using walltimer here 
// (so I need to create async spinner for that or multithreaded nodehandle??), 
// ANd what about the callbacks ?


} // namespace dai::rosBridge
