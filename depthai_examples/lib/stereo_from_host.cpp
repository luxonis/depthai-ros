#include <depthai_examples/stereo_from_host.hpp>


void StereoHost::initDepthaiDev(){
    
    // bool withDepth = true;
    bool outputDepth = true;
    bool outputRectified = false;
    bool lrcheck  = false;
    bool extended = false;
    bool subpixel = false;

    /**    xLinkInLeft 
     *                \
     *                 |-> stereo |-> xLinkOutDept
     *                / 
     *    xLinkInRight
     */

    auto xlinkInLeft  = _p.create<dai::node::XLinkIn>();
    auto xlinkInRight = _p.create<dai::node::XLinkIn>();
    auto stereo       = _p.create<dai::node::StereoDepth>();
    auto xoutDepth    = _p.create<dai::node::XLinkOut>();

    // auto xoutDisp    = _p.create<dai::node::XLinkOut>();
    // XLinkOut

    xlinkInLeft ->setStreamName("in_left");
    xlinkInRight->setStreamName("in_right");

    // xoutLeft ->setStreamName("left");
    // xoutRight->setStreamName("right");
    xoutDepth->setStreamName("depth");
    
    // // MonoCamera
    // monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    // monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    // //monoLeft->setFps(5.0);
    // monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    // monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    // //monoRight->setFps(5.0);

    // StereoDepth
    stereo->setOutputDepth(outputDepth);
    stereo->setOutputRectified(outputRectified);
    stereo->setConfidenceThreshold(200);
    stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout
    //stereo->loadCalibrationFile("../../../../depthai/resources/depthai.calib");
    //stereo->setInputResolution(1280, 720);
    // TODO: median filtering is disabled on device with (lrcheck || extended || subpixel)
    //stereo->setMedianFilter(dai::StereoDepthProperties::MedianFilter::MEDIAN_OFF);
    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);
    
    stereo->setInputResolution(1280, 720);
    // Link plugins CAM -> STEREO -> XLINK
    xlinkInLeft->out.link(stereo->left);
    xlinkInRight->out.link(stereo->right);

    // stereo->syncedLeft.link(xoutLeft->input);
    // stereo->syncedRight.link(xoutRight->input);
    // if(outputRectified)
    // {
    //     stereo->rectifiedLeft.link(xoutRectifL->input);
    //     stereo->rectifiedRight.link(xoutRectifR->input);
    // }
    // stereo->disparity.link(xoutDisp->input);
    stereo->depth.link(xoutDepth->input);

    // CONNECT TO DEVICE
     _dev = std::make_unique<dai::Device>(_p);
     _dev->startPipeline();

    _inImageStreams.push_back(_dev->getInputQueue("in_left"));
    _inImageStreams.push_back(_dev->getInputQueue("in_right"));

    _opImageStreams.push_back(_dev->getOutputQueue("depth", 30, false));
    
}

std::vector<std::shared_ptr<dai::DataOutputQueue>> StereoHost::getExposedOutputImageStreams(){
    return _opImageStreams;
}

std::vector<std::shared_ptr<dai::DataInputQueue>> StereoHost::getExposedInputImageStreams(){
    return _inImageStreams;
}
