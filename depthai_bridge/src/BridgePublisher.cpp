#include <depthai_bridge/BridgePublisher.hpp>
#include <thread>

namespace dai::ros {

template <class RosMsg, class SimMsg> 
BridgePublisher<RosMsg, SimMsg>::BridgePublisher(
    std::shared_ptr<dai::DataOutputQueue> daiMessageQueue, ros::NodeHandle &nh,
    std::string rosTopic, ConvertFunc converter, int queueSize, bool isMsgPtr = False)
    : _daiMessageQueue(daiMessageQueue), _nh(nh), _converter(converter),
      _rosTopic(rosTopic), _isMsgPtr(isMsgPtr) {
        
        if(isMsgPtr)
          _rosPublisher = _nh.advertise<RosMsg::SharedPtr>(rosTopic, queueSize);
        else
          _rosPublisher = _nh.advertise<RosMsg>(rosTopic, queueSize);
}

template <class RosMsg, class SimMsg> 
BridgePublisher<RosMsg, SimMsg>::startPublisherThread(){
    
    _readingThread = std::thread([&](){
    
        while(ros:ok()){
            auto daiDataPtr = _daiMessageQueue.tryGet();

            if(isMsgPtr){
              RosMsg::SharedPtr opMsgPtr = boost::make_shared<RosMsg>();
              _converter(daiDataPtr, *opMsgPtr);
              _rosPublisher.publish(opMsgPtr)
            }
            else{
              RosMsg::SharedPtr opMsg;
              _converter(daiDataPtr, opMsg);
              _rosPublisher.publish(opMsgPtr)
            }

        }
    
    });
}

template <class RosMsg, class SimMsg> 
BridgePublisher<RosMsg, SimMsg>::BridgePublisher(){
  _readingThread.join();
}

// TODO(sachin): alternative methods to publish would be using walltimer here 
// (so I need to create async spinner for that or multithreaded nodehandle??), 
// ANd what about the callbacks ?



} // namespace dai::ros
