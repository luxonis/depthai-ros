#pragma once

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

// pybind
// #include "pybind11_common.hpp"
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/datatype/IMUData.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_bridge/TrackedFeaturesConverter.hpp"
#include "image_transport/image_transport.hpp"
#include "pybind11/pybind11.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

namespace dai {
namespace ros {
namespace py = pybind11;

using namespace std::chrono_literals;
typedef std::map<std::string, std::string> remappingsMap;
// Node that produces messages.
class Producer : public rclcpp::Node {
   public:
    Producer(const std::string& name, rclcpp::NodeOptions options, const std::string& output) : Node(name, options) {
        // Create a publisher on the output topic.
        pub_ = this->create_publisher<std_msgs::msg::Int32>(output, 10);
        std::string test_param = this->declare_parameter<std::string>("test_param1", "default_value");
        RCLCPP_INFO(this->get_logger(), "test_param1: %s", test_param.c_str());
        std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
        // Create a timer which publishes on the output topic at ~1Hz.
        auto callback = [captured_pub]() -> void {
            auto pub_ptr = captured_pub.lock();
            if(!pub_ptr) {
                return;
            }
            static int32_t count = 0;
            std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32());
            msg->data = count++;
            printf("Published message with value: %d, and address: 0x%" PRIXPTR "\n", msg->data, reinterpret_cast<std::uintptr_t>(msg.get()));
            pub_ptr->publish(std::move(msg));
        };
        timer_ = this->create_wall_timer(1s, callback);
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// Node that consumes messages.
class Consumer : public rclcpp::Node {
   public:
    Consumer(const std::string& name, rclcpp::NodeOptions options, const std::string& input) : Node(name, options) {
        // Create a subscription on the input topic which prints on receipt of new messages.
        std::string test_param = this->declare_parameter<std::string>("test_param2", "default_value");
        RCLCPP_INFO(this->get_logger(), "test_param2: %s", test_param.c_str());
        sub_ = this->create_subscription<std_msgs::msg::Int32>(input, 10, [](std_msgs::msg::Int32::UniquePtr msg) {
            printf(" Received message with value: %d, and address: 0x%" PRIXPTR "\n", msg->data, reinterpret_cast<std::uintptr_t>(msg.get()));
        });
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

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
    void addNode(rclcpp::Node::SharedPtr node);
    void spin();

   private:
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> _executor;
    std::vector<rclcpp::Node::SharedPtr> _nodes;
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
    image_transport::CameraPublisher _pubCamera;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>> _pubCompressed;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>> _pubCamInfo;
    std::shared_ptr<ImageConverter> _imageConverter;
    sensor_msgs::msg::CameraInfo _camInfoMsg;
    bool _publishCompressed;
    bool _ipcEnabled;
};

class PYBIND11_EXPORT ImuStreamer {
   public:
    ImuStreamer(rclcpp::Node::SharedPtr node,
                const std::string& topicName,
                const std::string& frameName,
                ImuSyncMethod syncMode = ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL,
                double linear_accel_cov = 0.0,
                double angular_velocity_cov = 0.0,
                double rotation_cov = 0.0,
                double magnetic_field_cov = 0.0,
                bool enable_rotation = false,
                bool enable_magn = false,
                bool getBaseDeviceTimestamp = false);
    ~ImuStreamer() = default;
    void publish(const std::string& name, std::shared_ptr<dai::IMUData> imgFrame);

   private:
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> _pub;
    std::shared_ptr<ImuConverter> _imuConverter;
};

class PYBIND11_EXPORT SpatialDetectionStreamer {
   public:
    SpatialDetectionStreamer(rclcpp::Node::SharedPtr node,
                             const std::string& topicName,
                             std::string frameName,
                             int width,
                             int height,
                             bool normalized = false,
                             bool getBaseDeviceTimestamp = false);
    ~SpatialDetectionStreamer() = default;
    void publish(const std::string& name, std::shared_ptr<dai::SpatialImgDetections> detections);

   private:
    std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::Detection3DArray>> _pub;
    std::shared_ptr<SpatialDetectionConverter> _spatialDetectionConverter;
};

class PYBIND11_EXPORT DetectionStreamer {
   public:
    DetectionStreamer(rclcpp::Node::SharedPtr node,
                      const std::string& topicName,
                      std::string frameName,
                      int width,
                      int height,
                      bool normalized = false,
                      bool getBaseDeviceTimestamp = false);
    ~DetectionStreamer() = default;
    void publish(const std::string& name, std::shared_ptr<dai::ImgDetections> detections);

   private:
    std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::Detection2DArray>> _pub;
    std::shared_ptr<ImgDetectionConverter> _detectionConverter;
};

class PYBIND11_EXPORT TrackedFeaturesStreamer {
   public:
    TrackedFeaturesStreamer(rclcpp::Node::SharedPtr node, const std::string& topicName, std::string frameName, bool getBaseDeviceTimestamp = false);
    ~TrackedFeaturesStreamer() = default;
    void publish(const std::string& name, std::shared_ptr<dai::TrackedFeatures> trackedFeatures);

   private:
    std::shared_ptr<rclcpp::Publisher<depthai_ros_msgs::msg::TrackedFeatures>> _pub;
    std::shared_ptr<TrackedFeaturesConverter> _trackedFeaturesConverter;
};

struct PYBIND11_EXPORT RosBindings {
    static void bind(pybind11::module& m, void* pCallstack);
};
}  // namespace ros
}  // namespace dai