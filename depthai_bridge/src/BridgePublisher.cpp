#include <depthai_bridge/BridgePublisher.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <thread>

namespace dai::rosBridge {



template <class RosMsg, class SimMsg> 
BridgePublisher<RosMsg, SimMsg>::BridgePublisher(
    std::shared_ptr<dai::DataOutputQueue> daiMessageQueue, ros::NodeHandle &nh,
    std::string rosTopic, ConvertFunc &converter, int queueSize, bool isMsgPtr)
    : _daiMessageQueue(daiMessageQueue), _nh(nh), _converter(converter),
      _rosTopic(rosTopic), _isMsgPtr(isMsgPtr) {
        
        if(isMsgPtr)
          _rosPublisher = _nh.advertise<RosMsg::SharedPtr>(rosTopic, queueSize);
        else
          _rosPublisher = _nh.advertise<RosMsg>(rosTopic, queueSize);
}

template <class RosMsg, class SimMsg> 
void BridgePublisher<RosMsg, SimMsg>::startPublisherThread(){
    
    _readingThread = std::thread([&](){
    
        while(ros::ok()){
            auto daiDataPtr = _daiMessageQueue->tryGet();

            if(_isMsgPtr){
              // RosMsg::SharedPtr 
              boost::shared_ptr<RosMsg> opMsgPtr = boost::make_shared<RosMsg>();
              _converter(daiDataPtr, *opMsgPtr);
              _rosPublisher.publish(opMsgPtr);
            }
            else{
              RosMsg opMsg;
              _converter(daiDataPtr, opMsg);
              _rosPublisher.publish(opMsg);
            }

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
