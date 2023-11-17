#pragma once

// pybind
// #include "pybind11_common.hpp"
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "pybind11/pybind11.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace dai {
namespace ros {
namespace py = pybind11;
template <typename T, typename... Extra>
py::class_<T, std::shared_ptr<T>> message_class(py::module module, std::string class_name, const Extra&... extra) {
    py::class_<T, std::shared_ptr<T>> msg_class(module, class_name.c_str(), extra...);
    std::string ss = "Publisher_" + class_name;
    py::class_<rclcpp::Publisher<T>, std::shared_ptr<rclcpp::Publisher<T>>> pub_dr(module, ss.c_str());
    pub_dr.def("publish", [](std::shared_ptr<rclcpp::Publisher<T>> pub, const T& dr) { pub->publish(dr); });
    std::string s = "Subscription_" + class_name;
    py::class_<rclcpp::Subscription<T>, std::shared_ptr<rclcpp::Subscription<T>>> subscriber_image(module, s.c_str());

    msg_class.def_static(
        "__create_subscription__",
        [](rclcpp::Node::SharedPtr n, std::string topic, std::function<void(std::shared_ptr<T>)> callback) -> std::shared_ptr<rclcpp::Subscription<T>> {
            std::cout << "listening on '" << topic << "'" << std::endl;
            return n->create_subscription<T>(topic, 10, [callback](std::shared_ptr<T> im) { (callback)(im); });
        });

    msg_class.def_static("__create_publisher__", [](rclcpp::Node::SharedPtr n, std::string topic) { return n->create_publisher<T>(topic, 10); });
    return std::move(msg_class);
}
class PYBIND11_EXPORT ROSContextManager {
   public:
    ROSContextManager(){};
    ~ROSContextManager() = default;
    void init(py::list args);
    void shutdown();
    void spin(rclcpp::Node::SharedPtr node);

   private:
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> _executor;
    std::thread _executionThread;
};
class PYBIND11_EXPORT ImgStreamer {
   public:
    ImgStreamer(rclcpp::Node::SharedPtr node,
                dai::CalibrationHandler calibHandler,
                dai::CameraBoardSocket socket,
                const std::string& topicName,
                const std::string& frameName,
                bool interleaved,
                bool getBaseDeviceTimestamp = false);
    ~ImgStreamer() = default;
    void publish(const std::string& name, std::shared_ptr<dai::ImgFrame> imgFrame);
    void convertFromBitstream(dai::RawImgFrame::Type type);
   private:
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> _pub;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> _pubCamInfo;
    std::shared_ptr<ImageConverter> _imageConverter;
    sensor_msgs::msg::CameraInfo _camInfoMsg;
};
struct PYBIND11_EXPORT RosBindings {
    static void bind(pybind11::module& m, void* pCallstack);
};
}  // namespace ros
}  // namespace dai