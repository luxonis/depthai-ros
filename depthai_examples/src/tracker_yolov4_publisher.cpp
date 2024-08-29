#include <cstdio>
#include <iostream>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/TrackDetectionConverter.hpp"
#include "depthai_ros_msgs/msg/track_detection2_d_array.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/ObjectTracker.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"

const std::vector<std::string> label_map = {
    "person",        "bicycle",      "car",           "motorbike",     "aeroplane",   "bus",         "train",       "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",     "parking meter", "bench",       "bird",        "cat",         "dog",          "horse",
    "sheep",         "cow",          "elephant",      "bear",          "zebra",       "giraffe",     "backpack",    "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",       "skis",          "snowboard",   "sports ball", "kite",        "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket", "bottle",        "wine glass",  "cup",         "fork",        "knife",        "spoon",
    "bowl",          "banana",       "apple",         "sandwich",      "orange",      "broccoli",    "carrot",      "hot dog",      "pizza",
    "donut",         "cake",         "chair",         "sofa",          "pottedplant", "bed",         "diningtable", "toilet",       "tvmonitor",
    "laptop",        "mouse",        "remote",        "keyboard",      "cell phone",  "microwave",   "oven",        "toaster",      "sink",
    "refrigerator",  "book",         "clock",         "vase",          "scissors",    "teddy bear",  "hair drier",  "toothbrush"};

dai::Pipeline createPipeline(std::string nnPath, bool fullFrameTracking) {
    dai::Pipeline pipeline;
    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto detectionNetwork = pipeline.create<dai::node::YoloDetectionNetwork>();
    auto tracker = pipeline.create<dai::node::ObjectTracker>();

    // create xlink connections
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto xoutTracker = pipeline.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("preview");
    xoutTracker->setStreamName("tracklets");

    colorCam->setPreviewSize(416, 416);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    // setting node configs
    detectionNetwork->setBlobPath(nnPath);
    detectionNetwork->setConfidenceThreshold(0.3f);
    detectionNetwork->input.setBlocking(false);

    // yolo specific parameters
    detectionNetwork->setNumClasses(80);
    detectionNetwork->setCoordinateSize(4);
    detectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    detectionNetwork->setAnchorMasks({{"side13", {3, 4, 5}}, {"side26", {1, 2, 3}}});
    detectionNetwork->setIouThreshold(0.5f);

    // object tracker settings
    tracker->setTrackerType(dai::TrackerType::ZERO_TERM_COLOR_HISTOGRAM);
    tracker->setTrackerIdAssignmentPolicy(dai::TrackerIdAssignmentPolicy::UNIQUE_ID);

    // Link plugins CAM -> NN -> XLINK
    colorCam->preview.link(detectionNetwork->input);
    tracker->passthroughTrackerFrame.link(xoutRgb->input);

    if(fullFrameTracking) {
        colorCam->setPreviewKeepAspectRatio(false);
        colorCam->video.link(tracker->inputTrackerFrame);
        // tracker->inputTrackerFrame.setBlocking(false);
        //  do not block the pipeline if it's too slow on full frame
        // tracker->inputTrackerFrame.setQueueSize(2);
    } else {
        detectionNetwork->passthrough.link(tracker->inputTrackerFrame);
    }

    detectionNetwork->passthrough.link(tracker->inputDetectionFrame);
    detectionNetwork->out.link(tracker->inputDetections);
    tracker->out.link(xoutTracker->input);

    return pipeline;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("tracker_yolov4_node");

    std::string tfPrefix, resourceBaseFolder, nnPath;
    std::string camera_param_uri;
    bool fullFrameTracking;
    std::string nnName(BLOB_NAME);  // Set your blob name for the model here

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("camera_param_uri", camera_param_uri);
    node->declare_parameter("nnName", "");
    node->declare_parameter("resourceBaseFolder", "");
    node->declare_parameter("fullFrameTracking", false);

    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("camera_param_uri", camera_param_uri);
    node->get_parameter("resourceBaseFolder", resourceBaseFolder);
    node->get_parameter("fullFrameTracking", fullFrameTracking);

    if(resourceBaseFolder.empty()) {
        throw std::runtime_error("Send the path to the resouce folder containing NNBlob in \'resourceBaseFolder\' ");
    }

    std::string nnParam;
    node->get_parameter("nnName", nnParam);
    if(nnParam != "x") {
        node->get_parameter("nnName", nnName);
    }

    nnPath = resourceBaseFolder + "/" + nnName;
    dai::Pipeline pipeline = createPipeline(nnPath, fullFrameTracking);
    dai::Device device(pipeline);

    auto colorQueue = device.getOutputQueue("preview", 30, false);
    auto trackQueue = device.getOutputQueue("tracklets", 30, false);
    auto calibrationHandler = device.readCalibration();

    int width, height;
    auto boardName = calibrationHandler.getEepromData().boardName;
    if(height > 480 && boardName == "OAK-D-LITE") {
        width = 640;
        height = 480;
    }

    dai::rosBridge::ImageConverter rgbConverter(tfPrefix + "_rgb_camera_optical_frame", false);
    auto rgbCameraInfo = rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::CAM_A, -1, -1);
    dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rgbPublish(colorQueue,
                                                                                       node,
                                                                                       std::string("color/image"),
                                                                                       std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                                                                                                 &rgbConverter,  // since the converter has the same frame name
                                                                                                                 // and image type is also same we can reuse it
                                                                                                 std::placeholders::_1,
                                                                                                 std::placeholders::_2),
                                                                                       30,
                                                                                       rgbCameraInfo,
                                                                                       "color");

    dai::rosBridge::TrackDetectionConverter trackConverter(tfPrefix + "_rgb_camera_optical_frame", 416, 416, fullFrameTracking, 0.3);
    dai::rosBridge::BridgePublisher<depthai_ros_msgs::msg::TrackDetection2DArray, dai::Tracklets> trackPublish(
        trackQueue,
        node,
        std::string("color/yolov4_tracklets"),
        std::bind(&dai::rosBridge::TrackDetectionConverter::toRosMsg, &trackConverter, std::placeholders::_1, std::placeholders::_2),
        30);

    rgbPublish.addPublisherCallback();
    trackPublish.addPublisherCallback();

    rclcpp::spin(node);

    return 0;
}
