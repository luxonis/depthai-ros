#include <unistd.h>

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "depthai_ros_driver/camera.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#define GETPID getpid

#include "builtin_interfaces/msg/time.hpp"
#include "opencv2/opencv.hpp"

int encoding2mat_type(const std::string& encoding) {
    if(encoding == "mono8") {
        return CV_8UC1;
    } else if(encoding == "bgr8") {
        return CV_8UC3;
    } else if(encoding == "16UC1") {
        return CV_16SC1;
    } else if(encoding == "rgba8") {
        return CV_8UC4;
    }
    throw std::runtime_error("Unsupported mat type");
}

std::string mat_type2encoding(int mat_type) {
    switch(mat_type) {
        case CV_8UC1:
            return "mono8";
        case CV_8UC3:
            return "bgr8";
        case CV_16SC1:
            return "mono16";
        case CV_8UC4:
            return "rgba8";
        default:
            throw std::runtime_error("Unsupported encoding type");
    }
}

void set_now(builtin_interfaces::msg::Time& time) {
    std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
    if(now <= std::chrono::nanoseconds(0)) {
        time.sec = time.nanosec = 0;
    } else {
        time.sec = static_cast<builtin_interfaces::msg::Time::_sec_type>(now.count() / 1000000000);
        time.nanosec = now.count() % 1000000000;
    }
}

void draw_on_image(cv::Mat& image, const std::string& text, int height) {
    cv::Mat c_mat = image;
    cv::putText(c_mat, text.c_str(), cv::Point(10, height), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 0));
}

/// Node which receives sensor_msgs/Image messages and renders them using OpenCV.
class ImageViewNode : public rclcpp::Node {
   public:
    /// \brief Construct a new ImageViewNode for visualizing image data
    /// \param input The topic name to subscribe to
    /// \param node_name The node name to use
    /// \param watermark Whether to add a watermark to the image before displaying
    explicit ImageViewNode(const std::string& input, const std::string& node_name = "image_view_node", bool watermark = true)
        : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {
        // Create a subscription on the input topic.
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            input, rclcpp::SensorDataQoS(), [node_name, watermark](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                // Create a cv::Mat from the image message (without copying).
                cv::Mat cv_mat(msg->height, msg->width, encoding2mat_type(msg->encoding), const_cast<unsigned char*>(msg->data.data()));
                if(watermark) {
                    // Annotate with the pid and pointer address.
                    std::stringstream ss;
                    ss << "pid: " << GETPID() << ", ptr: " << msg.get();
                    draw_on_image(cv_mat, ss.str(), 60);
                }
                // Show the image.
                cv::Mat c_mat = cv_mat;
                cv::imshow(node_name.c_str(), c_mat);
                char key = cv::waitKey(1);  // Look for key presses.
                if(key == 27 /* ESC */ || key == 'q') {
                    rclcpp::shutdown();
                }
                if(key == ' ') {  // If <space> then pause until another <space>.
                    key = '\0';
                    while(key != ' ') {
                        key = cv::waitKey(1);
                        if(key == 27 /* ESC */ || key == 'q') {
                            rclcpp::shutdown();
                        }
                        if(!rclcpp::ok()) {
                            break;
                        }
                    }
                }
            });
    }

   private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

    cv::VideoCapture cap_;
    cv::Mat frame_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions opts = rclcpp::NodeOptions().use_intra_process_comms(true);
    auto camera_opts = opts;
    auto camera = std::make_unique<depthai_ros_driver::Camera>(camera_opts);
    auto view1 = std::make_shared<ImageViewNode>("/camera/rgb/image_raw");
    auto view2 = std::make_shared<ImageViewNode>("/camera/stereo/image_raw", "view2");

    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(camera->get_node_base_interface());
    executor.add_node(view1->get_node_base_interface());
    executor.add_node(view2->get_node_base_interface());

    executor.spin();

    rclcpp::shutdown();

    return 0;
}