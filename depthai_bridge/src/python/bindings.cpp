// https://gist.github.com/kervel/75d81b8c34e45a19706c661b90d02548
#include "depthai_bridge/python/bindings.hpp"

#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "depthai_bridge/ImageConverter.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "pybind11/pybind11.h"
#include "rclcpp/rclcpp.hpp"

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
    : _imageConverter(std::make_shared<ImageConverter>(frameName, interleaved, getBaseDeviceTimestamp)) {
    _pub = node->create_publisher<sensor_msgs::msg::Image>(topicName, 10);
    _pubCamInfo = node->create_publisher<sensor_msgs::msg::CameraInfo>(topicName + "/camera_info", 10);
    _camInfoMsg = _imageConverter->calibrationToCameraInfo(calibHandler, socket);
}
void ImgStreamer::publish(const std::string& name, std::shared_ptr<dai::ImgFrame> imgFrame) {
    auto imgMsg = _imageConverter->toRosMsgRawPtr(imgFrame);
    _camInfoMsg.header = imgMsg.header;
    _pub->publish(imgMsg);
    _pubCamInfo->publish(_camInfoMsg);
}

void ImgStreamer::convertFromBitstream(dai::RawImgFrame::Type type){
    _imageConverter->convertFromBitstream(type);
}

void ROSContextManager::init(py::list args) {
    std::vector<const char*> c_strs;
    for(auto& a : args) {
        c_strs.push_back(a.cast<std::string>().c_str());
    }
    rclcpp::init(c_strs.size(), c_strs.data());
}

void ROSContextManager::shutdown() {
    _executor->cancel();
    _executor.reset();
    rclcpp::shutdown();
}

void ROSContextManager::spin(rclcpp::Node::SharedPtr node) {
    _executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    _executor->add_node(node);
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
    py::class_<ImgStreamer, std::shared_ptr<ImgStreamer>> imgStreamer(m_ros, "ImgStreamer");
    py::class_<ROSContextManager, std::shared_ptr<ROSContextManager>> rosContextManager(m_ros, "ROSContextManager");


    node.def(py::init([](std::string nodename) { return std::make_shared<rclcpp::Node>(nodename); }));
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
    rosContextManager.def(py::init([]() { return std::make_shared<ROSContextManager>(); }));
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

    m_ros.def("ros_ok", [](){return rclcpp::ok();});

    m_ros.def("shutdown", []() { rclcpp::shutdown(); });
};
}  // namespace ros
}  // namespace dai