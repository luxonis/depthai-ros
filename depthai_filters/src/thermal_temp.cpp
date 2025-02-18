#include "depthai_filters/thermal_temp.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include "depthai_filters/utils.hpp"
#include "opencv2/highgui/highgui.hpp"

namespace depthai_filters {

ThermalTemp::ThermalTemp(const rclcpp::NodeOptions& options) : rclcpp::Node("thermal_temp", options) {
    onInit();
}
void ThermalTemp::onInit() {
    sub = this->create_subscription<sensor_msgs::msg::Image>("/thermal/raw_data/image_raw", 10, std::bind(&ThermalTemp::subCB, this, std::placeholders::_1));
    colorPub = this->create_publisher<sensor_msgs::msg::Image>("color", 10);
}

void ThermalTemp::mouseCallback(int /* event */, int x, int y, int /* flags */, void* /* userdata */) {
    mouseX = x;
    mouseY = y;
}

void ThermalTemp::subCB(const sensor_msgs::msg::Image::ConstSharedPtr& img) {
    const char* tempWindow = "temperature";
    cv::namedWindow(tempWindow, cv::WINDOW_NORMAL);
    cv::setMouseCallback(
        tempWindow,
        [](int event, int x, int y, int flags, void* userdata) {
            auto* self = static_cast<ThermalTemp*>(userdata);
            self->mouseCallback(event, x, y, flags, userdata);
        },
        this);
    cv::Mat frameFp32 = utils::msgToMat(this->get_logger(), img, sensor_msgs::image_encodings::TYPE_32FC1);
    cv::Mat normalized;
    cv::normalize(frameFp32, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::Mat colormapped;
    cv::applyColorMap(normalized, colormapped, cv::COLORMAP_MAGMA);
    if(mouseX < 0 || mouseY < 0 || mouseX >= colormapped.cols || mouseY >= colormapped.rows) {
        mouseX = std::max(0, std::min(static_cast<int>(mouseX), colormapped.cols - 1));
        mouseY = std::max(0, std::min(static_cast<int>(mouseY), colormapped.rows - 1));
    }
    double min, max;
    cv::minMaxLoc(frameFp32, &min, &max);
    auto textColor = cv::Scalar(255, 255, 255);
    // Draw crosshair
    cv::line(colormapped, cv::Point(mouseX - 10, mouseY), cv::Point(mouseX + 10, mouseY), textColor, 1);
    cv::line(colormapped, cv::Point(mouseX, mouseY - 10), cv::Point(mouseX, mouseY + 10), textColor, 1);
    // Draw deg C
    char text[32];
    snprintf(text, sizeof(text), "%.1f deg C", frameFp32.at<float>(mouseY, mouseX));
    bool putTextLeft = mouseX > colormapped.cols / 2;
    cv::putText(colormapped, text, cv::Point(putTextLeft ? mouseX - 100 : mouseX + 10, mouseY - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, textColor, 1);
    cv::imshow(tempWindow, colormapped);
    cv::waitKey(1);
    sensor_msgs::msg::Image outMsg;
    cv_bridge::CvImage(img->header, sensor_msgs::image_encodings::BGR8, colormapped).toImageMsg(outMsg);

    colorPub->publish(outMsg);
}

}  // namespace depthai_filters
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_filters::ThermalTemp);
