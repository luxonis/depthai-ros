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

  void startPublisherThread();
  ~BridgePublisher();
  
private:
  std::shared_ptr<dai::DataOutputQueue> _daiMessageQueue;
  ConvertFunc _converter;
  
  ros::NodeHandle _nh;
  ros::Publisher _rosPublisher;
  std::thread _readingThread;
  std::string _rosTopic;
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
void BridgePublisher<RosMsg, SimMsg>::startPublisherThread(){

    _readingThread = std::thread([&](){
        while(ros::ok()){
          auto daiDataPtr = _daiMessageQueue->tryGet<SimMsg>();
          if(daiDataPtr == nullptr) continue;

          RosMsg opMsg;
          _converter(daiDataPtr, opMsg);
          _rosPublisher.publish(opMsg);
            
        }
    });
}


template <class RosMsg, class SimMsg> 
BridgePublisher<RosMsg, SimMsg>::~BridgePublisher(){
  _readingThread.join();
}

// TODO(sachin): alternative methods to publish would be using walltimer here 
// (so I need to create async spinner for that or multithreaded nodehandle??), 
// ANd what about the callbacks ?


} // namespace dai::rosBridge
