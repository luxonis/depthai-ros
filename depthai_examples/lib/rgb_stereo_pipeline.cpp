#include <depthai_examples/rgb_stereo_pipeline.hpp>


void RGBStereoExampe::initDepthaiDev(){
    

    bool outputDepth = true;
    bool outputRectified = false;
    bool lrcheck  = false;
    bool extended = false;
    bool subpixel = false;

    auto monoLeft    = _p.create<dai::node::MonoCamera>();
    auto monoRight   = _p.create<dai::node::MonoCamera>();
    
    auto stereo      = _p.create<dai::node::StereoDepth>();
    auto xoutDepth   = _p.create<dai::node::XLinkOut>();

    xoutDepth  ->setStreamName("depth");
  
    // MonoCamera
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    //monoLeft->setFps(5.0);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    //monoRight->setFps(5.0);

    int maxDisp = 96;
    if (extended) maxDisp *= 2;
    if (subpixel) maxDisp *= 32; // 5 bits fractional disparity

    //     // StereoDepth
    stereo->setOutputDepth(outputDepth);
    stereo->setOutputRectified(outputRectified);
    stereo->setConfidenceThreshold(200);
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout

    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);

    // // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    stereo->depth.link(xoutDepth->input);


    // Color camers steream setup -------->
    auto colorCam = _p.create<dai::node::ColorCamera>();
    auto xlinkOut = _p.create<dai::node::XLinkOut>();
    xlinkOut->setStreamName("preview");
    
    colorCam->setPreviewSize(1920, 1080);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(true);

    // Link plugins CAM -> XLINK
    colorCam->preview.link(xlinkOut->input);


    // CONNECT TO DEVICE
    _dev = std::make_unique<dai::Device>(_p);
    _dev->startPipeline();

    _opImageStreams.push_back(_dev->getOutputQueue("depth", 30, false));
    _opImageStreams.push_back(_dev->getOutputQueue("preview", 30, true));

}

std::vector<std::shared_ptr<dai::DataOutputQueue>> RGBStereoExampe::getExposedImageStreams(){
        return _opImageStreams;
}
