#include <depthai_examples/rgb_pipeline.hpp>
#include "depthai/depthai.hpp"



void RgbCameraPipelineExample::initDepthaiDev(){

    auto colorCam = _p.create<dai::node::ColorCamera>();
    auto xlinkOut = _p.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("video");
    
    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);

    // Link plugins CAM -> XLINK
    colorCam->video.link(xlinkOut->input);
   
    // Connect to device with above created pipeline
    _dev = std::make_unique<dai::Device>(_p);
    // Start the pipeline
    _dev->startPipeline();
    _opImageStreams.push_back(_dev->getOutputQueue("video"));
}


std::vector<std::shared_ptr<dai::DataOutputQueue>> RgbCameraPipelineExample::getExposedImageStreams(){
        return _opImageStreams;
}
