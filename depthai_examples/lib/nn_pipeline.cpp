
#include <depthai_examples/nn_pipeline.hpp>
#include "depthai/depthai.hpp"

const std::vector<std::string> MobileNetDetectionExample::label_map = {"background",  "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",   "car",  "cat",   "chair",    "cow",
                                  "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"};


void MobileNetDetectionExample::initDepthaiDev(std::string nnPath){

    bool syncNN = true;
    auto colorCam = _p.create<dai::node::ColorCamera>();
    auto xlinkOut = _p.create<dai::node::XLinkOut>();
    auto detectionNetwork = _p.create<dai::node::MobileNetDetectionNetwork>();
    auto nnOut = _p.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("preview");
    nnOut->setStreamName("detections");

    colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->setFps(40);

    // testing MobileNet DetectionNetwork
    detectionNetwork->setConfidenceThreshold(0.5f);
    detectionNetwork->setBlobPath(nnPath);

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(detectionNetwork->input);
    if(syncNN) detectionNetwork->passthrough.link(xlinkOut->input);
    else colorCam->preview.link(xlinkOut->input);

    detectionNetwork->out.link(nnOut->input);

    _dev = std::make_unique<dai::Device>(_p);
    _dev->startPipeline();

    _opImageStreams.push_back(_dev->getOutputQueue("preview", 30, false));
    _opNNetStreams.push_back(_dev->getOutputQueue("detections", 30, false));

}


std::vector<std::shared_ptr<dai::DataOutputQueue>> MobileNetDetectionExample::getExposedImageStreams(){
    return _opImageStreams;
}


std::vector<std::shared_ptr<dai::DataOutputQueue>> MobileNetDetectionExample::getExposedNnetStreams(){
    return _opNNetStreams;
}