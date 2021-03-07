#pragma once

#include <thread>

#include "depthai/depthai.hpp"
#include "sensor_msgs/Image.h"
#include <depthai_ros_msgs/DetectionDaiArray.h>
#include <vision_msgs/Detection2DArray.h>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <thread>

#include "ros/ros.h"

namespace dai::rosBridge {

template <class RosMsg, class SimMsg> 
class BridgePublisher {
public:
  using ConvertFunc = std::function<void (std::shared_ptr<SimMsg> , RosMsg&)>;
  // using ConvertFuncPtr = std::function<void (const RosMsg &, SimMsg &)>;

  BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                  ros::NodeHandle &nh, std::string rosTopic,
                  ConvertFunc converter, int queueSize);

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
  std::thread _readingThread;
  std::string _rosTopic;
  bool isCallbackAdded = false;

};


template <class RosMsg, class SimMsg> 
BridgePublisher<RosMsg, SimMsg>::BridgePublisher(
    std::shared_ptr<dai::DataOutputQueue> daiMessageQueue, ros::NodeHandle &nh,
    std::string rosTopic, ConvertFunc converter, int queueSize)
    : _daiMessageQueue(daiMessageQueue), _nh(nh), _converter(converter),
      _rosTopic(rosTopic){
    _rosPublisher = _nh.advertise<RosMsg>(rosTopic, queueSize);
}

template <class RosMsg, class SimMsg> 
BridgePublisher<RosMsg, SimMsg>::BridgePublisher(const BridgePublisher& other){
  _daiMessageQueue = other._daiMessageQueue;
  _nh = other._nh;
  _converter = other._converter;
  _rosTopic = other._rosTopic;
  _rosPublisher = ros::Publisher(other._rosPublisher);

}


template <class RosMsg, class SimMsg> 
void BridgePublisher<RosMsg, SimMsg>::startPublisherThread(){
  if(isCallbackAdded){
    std::runtime_error("addPubisherCallback() function adds a callback to the"
                       "depthai which handles the publishing so no need to start" 
                       "the thread using startPublisherThread() ");
  }

  _readingThread = std::thread([&](){
    while(ros::ok()){
      auto daiDataPtr = _daiMessageQueue->tryGet<SimMsg>();
      if(daiDataPtr == nullptr) {
       std::cout << "No data found!!!";
        continue;
      }

      RosMsg opMsg;
      _converter(daiDataPtr, opMsg);

      if(_rosPublisher.getNumSubscribers() > 0) _rosPublisher.publish(opMsg);
        
    }
  });
}

template <class RosMsg, class SimMsg> 
void BridgePublisher<RosMsg, SimMsg>::daiCallback(std::string name, std::shared_ptr<ADatatype> data){

  auto daiDataPtr = std::dynamic_pointer_cast<SimMsg>(data);

  RosMsg opMsg;
  _converter(daiDataPtr, opMsg);
  if(_rosPublisher.getNumSubscribers() > 0) _rosPublisher.publish(opMsg);
      
}

template <class RosMsg, class SimMsg> 
void BridgePublisher<RosMsg, SimMsg>::addPubisherCallback(){

  _daiMessageQueue->addCallback(std::bind(&BridgePublisher<RosMsg, SimMsg>::daiCallback, this, std::placeholders::_1, std::placeholders::_2));
  isCallbackAdded = true;
}


template <class RosMsg, class SimMsg> 
BridgePublisher<RosMsg, SimMsg>::~BridgePublisher(){
  _readingThread.join();
}

// TODO(sachin): alternative methods to publish would be using walltimer here 
// (so I need to create async spinner for that or multithreaded nodehandle??), 
// ANd what about the callbacks ?


} // namespace dai::rosBridge
