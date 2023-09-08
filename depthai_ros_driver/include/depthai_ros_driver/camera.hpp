#pragma once

#include "depthai_bridge/TFPublisher.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/param_handlers/camera_param_handler.hpp"
#include "depthai_ros_driver/parametersConfig.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "dynamic_reconfigure/server.h"
#include "nodelet/nodelet.h"
#include "ros/node_handle.h"
#include "std_srvs/Trigger.h"

namespace dai {
class Pipeline;
class Device;
}  // namespace dai

namespace depthai_ros_driver {
using Trigger = std_srvs::Trigger;
class Camera : public nodelet::Nodelet {
   public:
    void onInit() override;
    /**
     * @brief Creates the pipeline and starts the device. Also sets up parameter callback and services.
     */
    void onConfigure();

   private:
    /**
     * @brief      Print information about the device type.
     */
    void getDeviceType();
    /**
     * @brief      Create the pipeline by using PipelineGenerator.
     */
    void createPipeline();

    /**
     * @brief      Connect either to a first available device or to a device with a specific USB port, MXID or IP. Loops continuously until a device is found.
     */
    void startDevice();
    /**
     * @brief      Sets up the queues and creates publishers for the nodes in the pipeline.
     */
    void setupQueues();
    /**
     * @brief Sets IR floodlight and dot pattern projector.
     */
    void setIR();
    /**
     * @brief Saves pipeline as a json to a file.
     */
    void savePipeline();
    /**
     * @brief Saves calibration data to a json file.
     */
    void saveCalib();
    /**
     * @brief Loads calibration data from a path.
     * @param path Path to the calibration file.
     */
    void loadCalib(const std::string& path);
    void parameterCB(parametersConfig& config, uint32_t level);
    std::shared_ptr<dynamic_reconfigure::Server<parametersConfig>> paramServer;
    std::unique_ptr<param_handlers::CameraParamHandler> ph;
    ros::ServiceServer startSrv, stopSrv, savePipelineSrv, saveCalibSrv;
    ros::Subscriber diagSub;
    /**
     * Closes all the queues, clears the configured BaseNodes, stops the pipeline and resets the device.
     */
    void stop();
    /**
     * Runs onConfigure();
     */
    void start();
    void restart();
    void diagCB(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg);

    bool startCB(Trigger::Request& /*req*/, Trigger::Response& res);
    bool stopCB(Trigger::Request& /*req*/, Trigger::Response& res);
    bool saveCalibCB(Trigger::Request& /*req*/, Trigger::Response& res);
    bool savePipelineCB(Trigger::Request& /*req*/, Trigger::Response& res);

    std::vector<std::string> usbStrings = {"UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS"};
    std::shared_ptr<dai::Pipeline> pipeline;
    std::shared_ptr<dai::Device> device;
    ros::NodeHandle pNH;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    bool camRunning = false;
    bool enableIR = false;
    double floodlightBrighness;
    double laserDotBrightness;
    std::unique_ptr<dai::ros::TFPublisher> tfPub;
};
}  // namespace depthai_ros_driver