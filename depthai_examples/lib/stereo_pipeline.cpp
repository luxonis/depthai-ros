#include <depthai_examples/stereo_pipeline.hpp>


void StereoExampe::initDepthaiDev(){
    

    bool withDepth = true;
    bool outputDepth = true;
    bool outputRectified = true;
    bool lrcheck  = false;
    bool extended = false;
    bool subpixel = false;

    auto monoLeft    = _p.create<dai::node::MonoCamera>();
    auto monoRight   = _p.create<dai::node::MonoCamera>();
    auto xoutLeft    = _p.create<dai::node::XLinkOut>();
    auto xoutRight   = _p.create<dai::node::XLinkOut>();
    auto stereo      = withDepth ? _p.create<dai::node::StereoDepth>() : nullptr;
    // auto xoutDisp    = _p.create<dai::node::XLinkOut>();
    auto xoutDepth   = _p.create<dai::node::XLinkOut>();
    auto xoutRectifL = _p.create<dai::node::XLinkOut>();
    auto xoutRectifR = _p.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    if (withDepth) {
        // xoutDisp   ->setStreamName("disparity");
        xoutDepth  ->setStreamName("depth");
        xoutRectifL->setStreamName("rectified_left");
        xoutRectifR->setStreamName("rectified_right");
    }

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

    if (withDepth) {
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

        // Link plugins CAM -> STEREO -> XLINK
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);

        stereo->syncedLeft.link(xoutLeft->input);
        stereo->syncedRight.link(xoutRight->input);
        if(outputRectified)
        {
            stereo->rectifiedLeft.link(xoutRectifL->input);
            stereo->rectifiedRight.link(xoutRectifR->input);
        }
        // stereo->disparity.link(xoutDisp->input);
        stereo->depth.link(xoutDepth->input);

    } else {
        // Link plugins CAM -> XLINK
        monoLeft->out.link(xoutLeft->input);
        monoRight->out.link(xoutRight->input);
    }

    // CONNECT TO DEVICE
     _dev = std::make_unique<dai::Device>(_p);
     _dev->startPipeline();

     _opImageStreams.push_back(_dev->getOutputQueue("left", 30, false));
     _opImageStreams.push_back(_dev->getOutputQueue("right", 30, false));
    //  if (withDepth) _opImageStreams.push_back(_dev->getOutputQueue("disparity", 30, false));
     if (withDepth) _opImageStreams.push_back(_dev->getOutputQueue("depth", 30, false));
     if (withDepth) _opImageStreams.push_back(_dev->getOutputQueue("rectified_left", 30, false));
     if (withDepth) _opImageStreams.push_back(_dev->getOutputQueue("rectified_right", 30, false));
    
}

std::vector<std::shared_ptr<dai::DataOutputQueue>> StereoExampe::getExposedImageStreams(){
        return _opImageStreams;
}
