
#include <cstdio>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/IMU.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/imu.hpp"

int main(int argc, char** argv) {
    std::string tfPrefix = "oak";
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared(tfPrefix);

    auto device = std::make_shared<dai::Device>();
    dai::Pipeline pipeline(device);

    // Define sources and outputs
    auto imu = pipeline.create<dai::node::IMU>();
    // Enable ACCELEROMETER_RAW at 480 hz rate
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 480);
    // Enable GYROSCOPE_RAW at 400 hz rate
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    // Set batch report threshold and max batch reports
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);

    // Create output queue
    auto imuQ = imu->out.createOutputQueue(8, false);
    pipeline.start();

    // Create a bridge publisher for RGB images
    depthai_bridge::ImuSyncMethod imuMode = depthai_bridge::ImuSyncMethod::COPY;
    auto imuConv = std::make_shared<depthai_bridge::ImuConverter>(depthai_bridge::getFrameName(tfPrefix, "imu_frame"), imuMode);

    auto calibrationHandler = device->readCalibration();
    auto tfPub =
        std::make_unique<depthai_bridge::TFPublisher>(node, calibrationHandler, device->getConnectedCameraFeatures(), tfPrefix, device->getDeviceName());

    auto imuPub = std::make_unique<depthai_bridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData>>(
        imuQ,
        node,
        "imu/data",
        [imuConv](std::shared_ptr<dai::IMUData> msg, std::deque<sensor_msgs::msg::Imu>& rosMsgs) { imuConv->toRosMsg(msg, rosMsgs); },
        30);

    imuPub->addPublisherCallback();

    while(rclcpp::ok() && pipeline.isRunning()) {
        rclcpp::spin(node);
    }

    return 0;
}
