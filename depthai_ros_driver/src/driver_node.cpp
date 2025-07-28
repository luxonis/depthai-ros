#include "depthai_ros_driver/driver.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto cam = std::make_shared<depthai_ros_driver::Driver>();

    rclcpp::spin(cam);
    rclcpp::shutdown();
    return 0;
}
