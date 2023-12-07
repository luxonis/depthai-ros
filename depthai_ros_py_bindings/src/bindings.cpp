// https://gist.github.com/kervel/75d81b8c34e45a19706c661b90d02548
#include "depthai_ros_py_bindings/bindings.hpp"

#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "depthai_bridge/ImageConverter.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "image_proc/rectify.hpp"
#include "ira_laser_tools/laserscan_multi_merger.hpp"
#include "laserscan_kinect/laserscan_kinect_node.hpp"
#include "pybind11/pybind11.h"
#include "rclcpp/rclcpp.hpp"
#include "rtabmap_slam/CoreWrapper.h"
#include "spectacularai_ros2/ros2_plugin.hpp"

namespace dai {
namespace ros {
namespace py = pybind11;

ImgStreamer::ImgStreamer(rclcpp::Node::SharedPtr node,
                         dai::CalibrationHandler calibHandler,
                         dai::CameraBoardSocket socket,
                         const std::string& topicName,
                         const std::string& frameName,
                         int width,
                         int height,
                         bool interleaved,
                         bool getBaseDeviceTimestamp)
    : _imageConverter(std::make_shared<ImageConverter>(frameName, interleaved, getBaseDeviceTimestamp)),
      _publishCompressed(false),
      _ipcEnabled(node->get_node_options().use_intra_process_comms()) {
    _imageConverter->setUpdateRosBaseTimeOnToRosMsg(true);
    if(_ipcEnabled) {
        _pub = node->create_publisher<sensor_msgs::msg::Image>(topicName, 10);
        _pubCompressed = node->create_publisher<sensor_msgs::msg::CompressedImage>(topicName + "/compressed", 10);
        _pubCamInfo = node->create_publisher<sensor_msgs::msg::CameraInfo>(topicName + "/camera_info", 10);
    } else {
        _pubCamera = image_transport::create_camera_publisher(node.get(), topicName);
    }
    _camInfoMsg = _imageConverter->calibrationToCameraInfo(calibHandler, socket, width, height);
}
void ImgStreamer::publish(const std::string& name, std::shared_ptr<dai::ImgFrame> imgFrame) {
    auto imgMsg = _imageConverter->toRosMsgRawPtr(imgFrame);
    _camInfoMsg.header = imgMsg.header;
    if(_ipcEnabled) {
        if(_publishCompressed) {
            sensor_msgs::msg::CompressedImage compressedImgMsg;
            compressedImgMsg.header = imgMsg.header;
            compressedImgMsg.format = "jpeg";
            size_t size = imgFrame->getData().size();
            compressedImgMsg.data.resize(size);
            compressedImgMsg.data.assign(imgFrame->getData().begin(), imgFrame->getData().end());
            _pubCompressed->publish(compressedImgMsg);
        }
        _pub->publish(imgMsg);
        _pubCamInfo->publish(_camInfoMsg);
    } else {
        _pubCamera.publish(imgMsg, _camInfoMsg);
    }
}

void ImgStreamer::convertFromBitstream(dai::RawImgFrame::Type type) {
    _imageConverter->convertFromBitstream(type);
    _publishCompressed = true;
}

ImuStreamer::ImuStreamer(rclcpp::Node::SharedPtr node,
                         const std::string& topicName,
                         const std::string& frameName,
                         ImuSyncMethod syncMode,
                         double linear_accel_cov,
                         double angular_velocity_cov,
                         double rotation_cov,
                         double magnetic_field_cov,
                         bool enable_rotation,
                         bool enable_magn,
                         bool getBaseDeviceTimestamp) {
    _imuConverter = std::make_shared<ImuConverter>(
        frameName, syncMode, linear_accel_cov, angular_velocity_cov, rotation_cov, magnetic_field_cov, enable_rotation, enable_magn, getBaseDeviceTimestamp);
    _imuConverter->setUpdateRosBaseTimeOnToRosMsg(true);
    _pub = node->create_publisher<sensor_msgs::msg::Imu>(topicName, 10);
}

void ImuStreamer::publish(const std::string& name, std::shared_ptr<dai::IMUData> imuFrame) {
    std::deque<sensor_msgs::msg::Imu> imuMsgs;
    _imuConverter->toRosMsg(imuFrame, imuMsgs);
    for(auto& msg : imuMsgs) {
        _pub->publish(msg);
    };
}

SpatialDetectionStreamer::SpatialDetectionStreamer(
    rclcpp::Node::SharedPtr node, const std::string& topicName, std::string frameName, int width, int height, bool normalized, bool getBaseDeviceTimestamp) {
    _spatialDetectionConverter = std::make_shared<SpatialDetectionConverter>(frameName, width, height, normalized, getBaseDeviceTimestamp);
    _pub = node->create_publisher<vision_msgs::msg::Detection3DArray>(topicName, 10);
}

void SpatialDetectionStreamer::publish(const std::string& name, std::shared_ptr<dai::SpatialImgDetections> detections) {
    std::deque<vision_msgs::msg::Detection3DArray> detectionMsg;
    _spatialDetectionConverter->toRosVisionMsg(detections, detectionMsg);
    for(auto& msg : detectionMsg) {
        _pub->publish(msg);
    };
}

DetectionStreamer::DetectionStreamer(
    rclcpp::Node::SharedPtr node, const std::string& topicName, std::string frameName, int width, int height, bool normalized, bool getBaseDeviceTimestamp) {
    _detectionConverter = std::make_shared<ImgDetectionConverter>(frameName, width, height, normalized, getBaseDeviceTimestamp);
    _pub = node->create_publisher<vision_msgs::msg::Detection2DArray>(topicName, 10);
}

void DetectionStreamer::publish(const std::string& name, std::shared_ptr<dai::ImgDetections> detections) {
    std::deque<vision_msgs::msg::Detection2DArray> detectionMsg;
    _detectionConverter->toRosMsg(detections, detectionMsg);
    for(auto& msg : detectionMsg) {
        _pub->publish(msg);
    };
}

TrackedFeaturesStreamer::TrackedFeaturesStreamer(rclcpp::Node::SharedPtr node,
                                                 const std::string& topicName,
                                                 std::string frameName,
                                                 bool getBaseDeviceTimestamp) {
    _trackedFeaturesConverter = std::make_shared<TrackedFeaturesConverter>(frameName, getBaseDeviceTimestamp);
    _pub = node->create_publisher<depthai_ros_msgs::msg::TrackedFeatures>(topicName, 10);
}

void TrackedFeaturesStreamer::publish(const std::string& name, std::shared_ptr<dai::TrackedFeatures> trackedFeatures) {
    std::deque<depthai_ros_msgs::msg::TrackedFeatures> trackedFeaturesMsg;
    _trackedFeaturesConverter->toRosMsg(trackedFeatures, trackedFeaturesMsg);
    for(auto& msg : trackedFeaturesMsg) {
        _pub->publish(msg);
    };
}

void ROSContextManager::init(py::list args, const std::string& executorType) {
    _executorType = executorType;
    std::vector<const char*> c_strs;
    for(auto& a : args) {
        c_strs.push_back(a.cast<std::string>().c_str());
    }
    rclcpp::init(c_strs.size(), c_strs.data());
    if(executorType == "single_threaded")
        _singleExecutor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    else if(executorType == "multi_threaded")
        _multiExecutor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
}

void ROSContextManager::shutdown() {
    if(_executorType == "single_threaded") {
        _singleExecutor->cancel();
        _singleExecutor.reset();
    } else if(_executorType == "multi_threaded") {
        _multiExecutor->cancel();
        _multiExecutor.reset();
    } else
        throw std::runtime_error("Unknown executor type");
    rclcpp::shutdown();
}

void ROSContextManager::addNode(rclcpp::Node::SharedPtr node) {
    if(_executorType == "single_threaded")
        _singleExecutor->add_node(node);
    else if(_executorType == "multi_threaded")
        _multiExecutor->add_node(node);
    else {
        throw std::runtime_error("Unknown executor type");
    }
}

void ROSContextManager::spin() {
    if(_executorType == "single_threaded") {
        auto spin_node = [this]() { _singleExecutor->spin(); };
        _executionThread = std::thread(spin_node);
        _executionThread.detach();
    } else if(_executorType == "multi_threaded") {
        auto spin_node = [this]() { _multiExecutor->spin(); };
        _executionThread = std::thread(spin_node);
        _executionThread.detach();
    } else
        throw std::runtime_error("Unknown executor type");
}

void RosBindings::bind(pybind11::module& m, void* pCallstack) {
    auto m_ros = m.def_submodule("ros", "ROS bindings");

    auto point = message_class<geometry_msgs::msg::Point>(m_ros, "Point");
    point.def(py::init<>());
    point.def_readwrite("x", &geometry_msgs::msg::Point::x);
    point.def_readwrite("y", &geometry_msgs::msg::Point::y);
    point.def_readwrite("z", &geometry_msgs::msg::Point::z);

    py::class_<rclcpp::Node, rclcpp::Node::SharedPtr> node(m_ros, "ROSNode");
    node.def(py::init(
        [](std::string nodename, const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) { return std::make_shared<rclcpp::Node>(nodename, options); }));
    py::class_<rclcpp::NodeOptions> nodeOptions(m_ros, "ROSNodeOptions");
    nodeOptions.def(
        py::init([](bool useIntraProcessComms = true, std::string nodeName = "", std::string paramFile = "", remappingsMap remappings = remappingsMap()) {
            rclcpp::NodeOptions options;
            options.use_intra_process_comms(useIntraProcessComms);
            std::vector<std::string> args;
            if(!paramFile.empty()) {
                args.push_back("--ros-args");
                args.push_back("--params-file");
                args.push_back(paramFile);
            }
            if(!remappings.empty()) {
                for(auto& remap : remappings) {
                    args.push_back("--remap");
                    args.push_back(remap.first + ":=" + remap.second);
                }
            }
            options.arguments(args);
            return options;
        }),
        py::arg("use_intra_process_comms") = true,
        py::arg("node_name") = "",
        py::arg("param_file") = "",
        py::arg("remappings") = remappingsMap());

    py::class_<Consumer, std::shared_ptr<Consumer>, rclcpp::Node> consumer(m_ros, "Consumer");
    consumer.def(py::init(
        [](std::string nodename, const rclcpp::NodeOptions& options, std::string input) { return std::make_shared<Consumer>(nodename, options, input); }));
    py::class_<Producer, std::shared_ptr<Producer>, rclcpp::Node> producer(m_ros, "Producer");
    producer.def(py::init(
        [](std::string nodename, const rclcpp::NodeOptions& options, std::string output) { return std::make_shared<Producer>(nodename, options, output); }));

    py::class_<ROSContextManager, std::shared_ptr<ROSContextManager>> rosContextManager(m_ros, "ROSContextManager");
    rosContextManager.def(py::init([]() { return std::make_shared<ROSContextManager>(); }));
    rosContextManager.def("add_node", &ROSContextManager::addNode);
    rosContextManager.def("init", &ROSContextManager::init);
    rosContextManager.def("shutdown", &ROSContextManager::shutdown);
    rosContextManager.def("spin", &ROSContextManager::spin);

    node.def("create_subscription", [](rclcpp::Node::SharedPtr n, py::object the_type, std::string topic, py::function callback) {
        auto f = the_type.attr("__create_subscription__");
        return f(n, topic, callback);
    });
    node.def("create_publisher", [](rclcpp::Node::SharedPtr n, py::object the_type, std::string topic) {
        auto f = the_type.attr("__create_publisher__");
        return f(n, topic);
    });

    node.def("log", [](rclcpp::Node::SharedPtr n, std::string logmsg) { RCLCPP_INFO(n->get_logger(), logmsg.c_str()); });

    m_ros.def("ros_ok", []() { return rclcpp::ok(); });

    m_ros.def("shutdown", []() { rclcpp::shutdown(); });

    py::class_<ImgStreamer, std::shared_ptr<ImgStreamer>> imgStreamer(m_ros, "ImgStreamer");

    imgStreamer.def(py::init([](rclcpp::Node::SharedPtr node,
                                dai::CalibrationHandler calibHandler,
                                dai::CameraBoardSocket socket,
                                const std::string& topicName,
                                const std::string& frameName,
                                int width,
                                int height,
                                bool interleaved,
                                bool getBaseDeviceTimestamp) {
                        return std::make_shared<ImgStreamer>(
                            node, calibHandler, socket, topicName, frameName, width, height, interleaved, getBaseDeviceTimestamp);
                    }),
                    py::arg("node"),
                    py::arg("calib_handler"),
                    py::arg("socket"),
                    py::arg("topic_name"),
                    py::arg("frame_name"),
                    py::arg("width") = -1,
                    py::arg("height") = -1,
                    py::arg("interleaved") = false,
                    py::arg("get_base_device_timestamp") = false);

    imgStreamer.def("publish", &ImgStreamer::publish);
    imgStreamer.def("convertFromBitStream", &ImgStreamer::convertFromBitstream);

    py::enum_<ImuSyncMethod>(m_ros, "ImuSyncMethod")
        .value("COPY", ImuSyncMethod::COPY)
        .value("LINEAR_INTERPOLATE_ACCEL", ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL)
        .value("LINEAR_INTERPOLATE_GYRO", ImuSyncMethod::LINEAR_INTERPOLATE_GYRO)
        .export_values();

    py::class_<ImuStreamer, std::shared_ptr<ImuStreamer>> imuStreamer(m_ros, "ImuStreamer");
    imuStreamer.def(py::init([](rclcpp::Node::SharedPtr node,
                                const std::string& topicName,
                                const std::string& frameName,
                                ImuSyncMethod syncMode,
                                double linear_accel_cov,
                                double angular_velocity_cov,
                                double rotation_cov,
                                double magnetic_field_cov,
                                bool enable_rotation,
                                bool enable_magn,
                                bool getBaseDeviceTimestamp) {
                        return std::make_shared<ImuStreamer>(node,
                                                             topicName,
                                                             frameName,
                                                             syncMode,
                                                             linear_accel_cov,
                                                             angular_velocity_cov,
                                                             rotation_cov,
                                                             magnetic_field_cov,
                                                             enable_rotation,
                                                             enable_magn,
                                                             getBaseDeviceTimestamp);
                    }),
                    py::arg("node"),
                    py::arg("topic_name"),
                    py::arg("frame_name"),
                    py::arg("sync_mode") = ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL,
                    py::arg("linear_accel_cov") = 0.0,
                    py::arg("angular_velocity_cov") = 0.0,
                    py::arg("rotation_cov") = 0.0,
                    py::arg("magnetic_field_cov") = 0.0,
                    py::arg("enable_rotation") = false,
                    py::arg("enable_magn") = false,
                    py::arg("get_base_device_timestamp") = false);
    imuStreamer.def("publish", &ImuStreamer::publish);

    py::class_<SpatialDetectionStreamer, std::shared_ptr<SpatialDetectionStreamer>> spatialDetectionStreamer(m_ros, "SpatialDetectionStreamer");
    spatialDetectionStreamer.def(py::init([](rclcpp::Node::SharedPtr node,
                                             const std::string& topicName,
                                             std::string frameName,
                                             int width,
                                             int height,
                                             bool normalized = false,
                                             bool getBaseDeviceTimestamp = false) {
                                     return std::make_shared<SpatialDetectionStreamer>(
                                         node, topicName, frameName, width, height, normalized, getBaseDeviceTimestamp);
                                 }),
                                 py::arg("node"),
                                 py::arg("topic_name"),
                                 py::arg("frame_name"),
                                 py::arg("width"),
                                 py::arg("height"),
                                 py::arg("normalized") = false,
                                 py::arg("get_base_device_timestamp") = false);
    spatialDetectionStreamer.def("publish", &SpatialDetectionStreamer::publish);

    py::class_<DetectionStreamer, std::shared_ptr<DetectionStreamer>> detectionStreamer(m_ros, "DetectionStreamer");
    detectionStreamer.def(py::init([](rclcpp::Node::SharedPtr node,
                                      const std::string& topicName,
                                      std::string frameName,
                                      int width,
                                      int height,
                                      bool normalized = false,
                                      bool getBaseDeviceTimestamp = false) {
                              return std::make_shared<DetectionStreamer>(node, topicName, frameName, width, height, normalized, getBaseDeviceTimestamp);
                          }),
                          py::arg("node"),
                          py::arg("topic_name"),
                          py::arg("frame_name"),
                          py::arg("width"),
                          py::arg("height"),
                          py::arg("normalized") = false,
                          py::arg("get_base_device_timestamp") = false);
    detectionStreamer.def("publish", &DetectionStreamer::publish);

    py::class_<TrackedFeaturesStreamer, std::shared_ptr<TrackedFeaturesStreamer>> trackedFeaturesStreamer(m_ros, "TrackedFeaturesStreamer");
    trackedFeaturesStreamer.def(
        py::init([](rclcpp::Node::SharedPtr node, const std::string& topicName, std::string frameName, bool getBaseDeviceTimestamp = false) {
            return std::make_shared<TrackedFeaturesStreamer>(node, topicName, frameName, getBaseDeviceTimestamp);
        }),
        py::arg("node"),
        py::arg("topic_name"),
        py::arg("frame_name"),
        py::arg("get_base_device_timestamp") = false);
    trackedFeaturesStreamer.def("publish", &TrackedFeaturesStreamer::publish);

    py::class_<rtabmap_slam::CoreWrapper, std::shared_ptr<rtabmap_slam::CoreWrapper>, rclcpp::Node> RTABMapCoreWrapper(m_ros, "RTABMapCoreWrapper");
    RTABMapCoreWrapper.def(py::init([](const rclcpp::NodeOptions& options) { return std::make_shared<rtabmap_slam::CoreWrapper>(options); }));
    py::class_<spectacularAI::ros2::Node, std::shared_ptr<spectacularAI::ros2::Node>, rclcpp::Node> SpectacularAINode(m_ros, "SpectacularAINode");
    SpectacularAINode.def(py::init([](const rclcpp::NodeOptions& options) { return std::make_shared<spectacularAI::ros2::Node>(options); }));

    py::class_<image_proc::RectifyNode, std::shared_ptr<image_proc::RectifyNode>, rclcpp::Node> ImageProcRectifyNode(m_ros, "ImageProcRectifyNode");
    ImageProcRectifyNode.def(py::init([](const rclcpp::NodeOptions& options) { return std::make_shared<image_proc::RectifyNode>(options); }));
    py::class_<laserscan_kinect::LaserScanKinectNode, std::shared_ptr<laserscan_kinect::LaserScanKinectNode>, rclcpp::Node> LaserScanKinectNode(
        m_ros, "LaserScanKinectNode");
    LaserScanKinectNode.def(py::init([](const rclcpp::NodeOptions& options) { return std::make_shared<laserscan_kinect::LaserScanKinectNode>(options); }));
    py::class_<ira_laser_tools::LaserscanMerger, std::shared_ptr<ira_laser_tools::LaserscanMerger>, rclcpp::Node> LaserscanMerger(m_ros, "LaserscanMerger");
    LaserscanMerger.def(py::init([](const rclcpp::NodeOptions& options) { return std::make_shared<ira_laser_tools::LaserscanMerger>(options); }));
};
}  // namespace ros
}  // namespace dai