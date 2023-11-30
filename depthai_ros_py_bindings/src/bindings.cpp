// https://gist.github.com/kervel/75d81b8c34e45a19706c661b90d02548
#include "depthai_ros_py_bindings/bindings.hpp"

#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "depthai_bridge/ImageConverter.hpp"
#include "geometry_msgs/msg/point.hpp"
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
                         bool interleaved,
                         bool getBaseDeviceTimestamp)
    : _imageConverter(std::make_shared<ImageConverter>(frameName, interleaved, getBaseDeviceTimestamp)),
      _publishCompressed(false),
      _ipcEnabled(node->get_node_options().use_intra_process_comms()) {
    if(_ipcEnabled) {
        _pub = node->create_publisher<sensor_msgs::msg::Image>(topicName, 10);
        _pubCompressed = node->create_publisher<sensor_msgs::msg::CompressedImage>(topicName + "/compressed", 10);
        _pubCamInfo = node->create_publisher<sensor_msgs::msg::CameraInfo>(topicName + "/camera_info", 10);
    } else {
        _pubCamera = image_transport::create_camera_publisher(node.get(), topicName + "/image_raw");
    }
    _camInfoMsg = _imageConverter->calibrationToCameraInfo(calibHandler, socket);
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

void ROSContextManager::init(py::list args) {
    std::vector<const char*> c_strs;
    for(auto& a : args) {
        c_strs.push_back(a.cast<std::string>().c_str());
    }
    rclcpp::init(c_strs.size(), c_strs.data());
    _executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
}

void ROSContextManager::shutdown() {
    _executor->cancel();
    _executor.reset();
    rclcpp::shutdown();
}

void ROSContextManager::addNode(rclcpp::Node::SharedPtr node) {
    _executor->add_node(node);
}

void ROSContextManager::spin() {
    auto spin_node = [this]() { _executor->spin(); };
    _executionThread = std::thread(spin_node);
    _executionThread.detach();
}

void RosBindings::bind(pybind11::module& m, void* pCallstack) {
    auto m_ros = m.def_submodule("ros", "ROS bindings");

    auto point = message_class<geometry_msgs::msg::Point>(m_ros, "Point");
    point.def(py::init<>());
    point.def_readwrite("x", &geometry_msgs::msg::Point::x);
    point.def_readwrite("y", &geometry_msgs::msg::Point::y);
    point.def_readwrite("z", &geometry_msgs::msg::Point::z);

    py::class_<rclcpp::Node, rclcpp::Node::SharedPtr> node(m_ros, "ROSNode");
    node.def(
        py::init([](std::string nodename, rclcpp::NodeOptions options = rclcpp::NodeOptions()) { return std::make_shared<rclcpp::Node>(nodename, options); }));
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
                for (auto& remap : remappings) {
                    args.push_back("--remap");
                    args.push_back(remap.first + ":=" + remap.second);
                }
            }
            options.arguments(args);
            return options;
        }));

    py::class_<Consumer, std::shared_ptr<Consumer>, rclcpp::Node> consumer(m_ros, "Consumer");
    consumer.def(
        py::init([](std::string nodename, rclcpp::NodeOptions options, std::string input) { return std::make_shared<Consumer>(nodename, options, input); }));
    py::class_<Producer, std::shared_ptr<Producer>, rclcpp::Node> producer(m_ros, "Producer");
    producer.def(
        py::init([](std::string nodename, rclcpp::NodeOptions options, std::string output) { return std::make_shared<Producer>(nodename, options, output); }));

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
                                bool interleaved,
                                bool getBaseDeviceTimestamp) {
        return std::make_shared<ImgStreamer>(node, calibHandler, socket, topicName, frameName, interleaved, getBaseDeviceTimestamp);
    }));

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
    }));
    imuStreamer.def("publish", &ImuStreamer::publish);

    py::class_<SpatialDetectionStreamer, std::shared_ptr<SpatialDetectionStreamer>> spatialDetectionStreamer(m_ros, "SpatialDetectionStreamer");
    spatialDetectionStreamer.def(py::init([](rclcpp::Node::SharedPtr node,
                                             const std::string& topicName,
                                             std::string frameName,
                                             int width,
                                             int height,
                                             bool normalized = false,
                                             bool getBaseDeviceTimestamp = false) {
        return std::make_shared<SpatialDetectionStreamer>(node, topicName, frameName, width, height, normalized, getBaseDeviceTimestamp);
    }));
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
    }));
    detectionStreamer.def("publish", &DetectionStreamer::publish);

    py::class_<TrackedFeaturesStreamer, std::shared_ptr<TrackedFeaturesStreamer>> trackedFeaturesStreamer(m_ros, "TrackedFeaturesStreamer");
    trackedFeaturesStreamer.def(
        py::init([](rclcpp::Node::SharedPtr node, const std::string& topicName, std::string frameName, bool getBaseDeviceTimestamp = false) {
            return std::make_shared<TrackedFeaturesStreamer>(node, topicName, frameName, getBaseDeviceTimestamp);
        }));
    trackedFeaturesStreamer.def("publish", &TrackedFeaturesStreamer::publish);

    py::class_<rtabmap_slam::CoreWrapper, std::shared_ptr<rtabmap_slam::CoreWrapper>, rclcpp::Node> RTABMapCoreWrapper(m_ros, "RTABMapCoreWrapper");
    RTABMapCoreWrapper.def(py::init([](rclcpp::NodeOptions options) { 
        return std::make_shared<rtabmap_slam::CoreWrapper>(options); }));
    py::class_<spectacularAI::ros2::Node, std::shared_ptr<spectacularAI::ros2::Node>, rclcpp::Node> SpectacularAINode(m_ros, "SpectacularAINode");
    SpectacularAINode.def(py::init([](rclcpp::NodeOptions options) { 
        return std::make_shared<spectacularAI::ros2::Node>(options); }));
};
}  // namespace ros
}  // namespace dai