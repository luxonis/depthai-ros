#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>

#include <camera_info_manager/camera_info_manager.h>
#include <functional>
#include <depthai_examples/stereo_from_host.hpp>

// #include <depthai_examples/daiUtility.hpp>
// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/BridgePublisher.hpp>

using ImagePublisher = dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>;

class StereoMimicNode{

    public:
    StereoMimicNode(ros::NodeHandle nh): 
                                _nh(nh), 
                                _pnh("~"),
                                _leftImageSub(_nh, "left/image", 10),
                                _rightImageSub(_nh, "right/image", 10),
                                _sync(_leftImageSub, _rightImageSub, 15)
    {
        std::string deviceName;
        std::string camera_param_uri;
        int bad_params = 0;

        bad_params += !_pnh.getParam("camera_name", deviceName);
        bad_params += !_pnh.getParam("camera_param_uri", camera_param_uri);

        _stereoPipeline = std::make_unique<StereoHost>();
        _stereoPipeline->initDepthaiDev();
        _outputQueues = _stereoPipeline->getExposedOutputImageStreams();
        _inputQueues  = _stereoPipeline->getExposedInputImageStreams();

        std::string stereo_uri = camera_param_uri + "/" + "right.yaml";
        _outputConverter = std::make_unique<dai::rosBridge::ImageConverter>(deviceName + "_right_camera_optical_frame", true);
        _imagePublisher  = std::make_unique<ImagePublisher>(_outputQueues[0],
                                                           _pnh, 
                                                           std::string("stereo/image"),
                                                           std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                           _outputConverter.get(), 
                                                           std::placeholders::_1, 
                                                           std::placeholders::_2) , 
                                                           30,
                                                           stereo_uri,
                                                           "stereo");

        _imagePublisher->addPubisherCallback();
        _inputConverter = std::make_unique<dai::rosBridge::ImageConverter>(true);
        _sync.registerCallback(boost::bind(&StereoMimicNode::stereoCallback, this, _1, _2));
        
        ROS_INFO("Constructyer used to ctreated xxxxxxxxxxxxxxxx");
    }

    void stereoCallback(const sensor_msgs::ImageConstPtr& leftImageMsg, const sensor_msgs::ImageConstPtr& rightImageMsg){
        
        std::cout << " In callbacks: " << std::endl;

        dai::ImgFrame leftFrame;
        dai::ImgFrame rightFrame;

        _inputConverter->toDaiMsg(*leftImageMsg, leftFrame);
        _inputConverter->toDaiMsg(*rightImageMsg, rightFrame);

        _inputQueues[0]->send(leftFrame);
        _inputQueues[1]->send(rightFrame);

        return;

    }

    private:
    std::unique_ptr<StereoHost> _stereoPipeline;
    std::unique_ptr<ImagePublisher> _imagePublisher;
    std::unique_ptr<dai::rosBridge::ImageConverter> _outputConverter;
    std::unique_ptr<dai::rosBridge::ImageConverter> _inputConverter;
    
    ros::NodeHandle _pnh;
    ros::NodeHandle _nh;
    
    std::vector<std::shared_ptr<dai::DataOutputQueue>> _outputQueues;
    std::vector<std::shared_ptr<dai::DataInputQueue>> _inputQueues;
    
    message_filters::Subscriber<sensor_msgs::Image> _leftImageSub;
    message_filters::Subscriber<sensor_msgs::Image> _rightImageSub;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> _sync;
};


int main(int argc, char** argv){

    ros::init(argc, argv, "stereo_mimic_node");
    ros::NodeHandle nh;
    StereoMimicNode nodeImp(nh);

    // std::cout << "------------HI HI----------: " << std::endl;

    ros::spin();

    return 0;
}


