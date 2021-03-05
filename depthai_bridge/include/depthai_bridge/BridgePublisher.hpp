#pragma once

#include <thread>

#include "depthai/depthai.hpp"
#include "sensor_msgs/Image.h"
#include <depthai_ros_msgs/DetectionDaiArray.h>
#include <vision_msgs/Detection2DArray.h>

#include "ros/ros.h"

namespace dai::rosBridge {

template <class RosMsg, class SimMsg> 
class BridgePublisher {
public:
  using ConvertFunc = std::function<void (std::shared_ptr<SimMsg> , RosMsg &)>;
  // using ConvertFuncPtr = std::function<void (const RosMsg &, SimMsg &)>;

  BridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                  ros::NodeHandle &nh, std::string rosTopic,
                  ConvertFunc &converter, int queueSize, bool isMsgPtr = false);

  void startPublisherThread();
  ~BridgePublisher();
  
private:
  std::shared_ptr<dai::DataOutputQueue> _daiMessageQueue;
  ConvertFunc _converter;
  
  ros::NodeHandle _nh;
  ros::Publisher _rosPublisher;
  std::thread _readingThread;
  bool _isMsgPtr;
  std::string _rosTopic;
};
} // namespace dai::rosBridge
