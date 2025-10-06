#include "depthai_ros_driver/pipeline/base_types.hpp"

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai_bridge/depthaiUtility.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/nn/nn_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/nn/spatial_nn_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/imu.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/rgbd.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
// #include "depthai_ros_driver/dai_nodes/sensors/slam.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/stereo.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/thermal.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/tof.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/vio.hpp"
#include "depthai_ros_driver/pipeline/base_pipeline.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace pipeline_gen {

std::vector<std::unique_ptr<dai_nodes::BaseNode>> RGB::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                      std::shared_ptr<dai::Device> device,
                                                                      std::shared_ptr<dai::Pipeline> pipeline,
                                                                      std::shared_ptr<param_handlers::PipelineGenParamHandler> ph,
                                                                      const std::string& deviceName,
                                                                      bool rsCompat,
                                                                      const std::string& nnType) {
    using namespace dai_nodes::sensor_helpers;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto rgb =
        std::make_unique<dai_nodes::SensorWrapper>(getNodeName(node, NodeNameEnum::RGB), node, pipeline, deviceName, rsCompat, dai::CameraBoardSocket::CAM_A);
    addNnNode(daiNodes, node, pipeline, deviceName, rsCompat, *rgb, nnType);
    daiNodes.push_back(std::move(rgb));
    if(checkForImu(ph, device, node->get_logger())) {
        auto imu = std::make_unique<dai_nodes::Imu>("imu", node, pipeline, device, rsCompat);
        daiNodes.push_back(std::move(imu));
    }
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> RGBD::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                       std::shared_ptr<dai::Device> device,
                                                                       std::shared_ptr<dai::Pipeline> pipeline,
                                                                       std::shared_ptr<param_handlers::PipelineGenParamHandler> ph,
                                                                       const std::string& deviceName,
                                                                       bool rsCompat,
                                                                       const std::string& nnType) {
    using namespace dai_nodes::sensor_helpers;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto rgb =
        std::make_unique<dai_nodes::SensorWrapper>(getNodeName(node, NodeNameEnum::RGB), node, pipeline, deviceName, rsCompat, dai::CameraBoardSocket::CAM_A);
    auto stereo = std::make_unique<dai_nodes::Stereo>(getNodeName(node, NodeNameEnum::Stereo), node, pipeline, device, rsCompat);
    if(stereo->isAligned() && stereo->getSocketID() == rgb->getSocketID()) {
        rgb->getDefaultOut()->link(stereo->getInput(static_cast<int>(dai_nodes::link_types::StereoLinkType::align)));
    }

    addNnNode(daiNodes, node, pipeline, deviceName, rsCompat, *rgb, *stereo, nnType);
    addRgbdNode(daiNodes, node, device, pipeline, ph, rsCompat, *rgb, *stereo);
    if(checkForImu(ph, device, node->get_logger())) {
        auto imu = std::make_unique<dai_nodes::Imu>("imu", node, pipeline, device, rsCompat);
        if(ph->getParam<bool>("i_enable_vio")) {
            auto vio = std::make_unique<dai_nodes::Vio>("vio", node, pipeline, device, rsCompat, *stereo, *imu);
            // if(ph->getParam<bool>("i_enable_slam")) {
            //     std::unique_ptr<dai_nodes::Slam> slam;
            //     if(stereo->getSocketID() == stereo->getLeftSensor()->getSocketID()) {
            //         slam = std::make_unique<dai_nodes::Slam>("slam", node, pipeline, device, rsCompat, *stereo->getLeftSensor(), *vio, *stereo);
            //     } else if(stereo->getSocketID() == stereo->getRightSensor()->getSocketID()) {
            //         slam = std::make_unique<dai_nodes::Slam>("slam", node, pipeline, device, rsCompat, *stereo->getRightSensor(), *vio, *stereo);
            //     } else if(stereo->getSocketID() == rgb->getSocketID()) {
            //         slam = std::make_unique<dai_nodes::Slam>("slam", node, pipeline, device, rsCompat, *rgb, *vio, *stereo);
            //     } else {
            //         throw std::runtime_error("Stereo socket is not left, right or rgb. Cannot create SLAM node.");
            //     }
            //     daiNodes.push_back(std::move(slam));
            // }
            daiNodes.push_back(std::move(vio));
        }
        daiNodes.push_back(std::move(imu));
    }

    daiNodes.push_back(std::move(rgb));
    daiNodes.push_back(std::move(stereo));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> RGBStereo::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                            std::shared_ptr<dai::Device> device,
                                                                            std::shared_ptr<dai::Pipeline> pipeline,
                                                                            std::shared_ptr<param_handlers::PipelineGenParamHandler> ph,
                                                                            const std::string& deviceName,
                                                                            bool rsCompat,
                                                                            const std::string& nnType) {
    using namespace dai_nodes::sensor_helpers;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto rgb =
        std::make_unique<dai_nodes::SensorWrapper>(getNodeName(node, NodeNameEnum::RGB), node, pipeline, deviceName, rsCompat, dai::CameraBoardSocket::CAM_A);
    auto left =
        std::make_unique<dai_nodes::SensorWrapper>(getNodeName(node, NodeNameEnum::Left), node, pipeline, deviceName, rsCompat, dai::CameraBoardSocket::CAM_B);
    auto right =
        std::make_unique<dai_nodes::SensorWrapper>(getNodeName(node, NodeNameEnum::Right), node, pipeline, deviceName, rsCompat, dai::CameraBoardSocket::CAM_C);
    addNnNode(daiNodes, node, pipeline, deviceName, rsCompat, *rgb, nnType);
    if(checkForImu(ph, device, node->get_logger())) {
        auto imu = std::make_unique<dai_nodes::Imu>("imu", node, pipeline, device, rsCompat);
        if(ph->getParam<bool>("i_enable_vio")) {
            auto vio = std::make_unique<dai_nodes::Vio>("vio", node, pipeline, device, rsCompat, *left, *right, *imu);
            daiNodes.push_back(std::move(vio));
        }
        daiNodes.push_back(std::move(imu));
    }
    daiNodes.push_back(std::move(rgb));
    daiNodes.push_back(std::move(left));
    daiNodes.push_back(std::move(right));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> Stereo::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                         std::shared_ptr<dai::Device> device,
                                                                         std::shared_ptr<dai::Pipeline> pipeline,
                                                                         std::shared_ptr<param_handlers::PipelineGenParamHandler> ph,
                                                                         const std::string& deviceName,
                                                                         bool rsCompat,
                                                                         const std::string& /*nnType*/) {
    using namespace dai_nodes::sensor_helpers;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto left =
        std::make_unique<dai_nodes::SensorWrapper>(getNodeName(node, NodeNameEnum::Left), node, pipeline, deviceName, rsCompat, dai::CameraBoardSocket::CAM_B);
    auto right =
        std::make_unique<dai_nodes::SensorWrapper>(getNodeName(node, NodeNameEnum::Right), node, pipeline, deviceName, rsCompat, dai::CameraBoardSocket::CAM_C);
    if(checkForImu(ph, device, node->get_logger())) {
        auto imu = std::make_unique<dai_nodes::Imu>("imu", node, pipeline, device, rsCompat);
        if(ph->getParam<bool>("i_enable_vio")) {
            auto vio = std::make_unique<dai_nodes::Vio>("vio", node, pipeline, device, rsCompat, *left, *right, *imu);
            daiNodes.push_back(std::move(vio));
        }
        daiNodes.push_back(std::move(imu));
    }
    daiNodes.push_back(std::move(left));
    daiNodes.push_back(std::move(right));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> Depth::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                        std::shared_ptr<dai::Device> device,
                                                                        std::shared_ptr<dai::Pipeline> pipeline,
                                                                        std::shared_ptr<param_handlers::PipelineGenParamHandler> ph,
                                                                        const std::string& /* deviceName */,
                                                                        bool rsCompat,
                                                                        const std::string& /*nnType*/) {
    using namespace dai_nodes::sensor_helpers;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto stereo = std::make_unique<dai_nodes::Stereo>(getNodeName(node, NodeNameEnum::Stereo), node, pipeline, device, rsCompat);
    if(checkForImu(ph, device, node->get_logger())) {
        auto imu = std::make_unique<dai_nodes::Imu>("imu", node, pipeline, device, rsCompat);
        if(ph->getParam<bool>("i_enable_vio")) {
            auto vio = std::make_unique<dai_nodes::Vio>("vio", node, pipeline, device, rsCompat, *stereo, *imu);
            // if(ph->getParam<bool>("i_enable_slam")) {
            //     std::unique_ptr<dai_nodes::Slam> slam;
            //     if(stereo->getSocketID() == stereo->getLeftSensor()->getSocketID()) {
            //         slam = std::make_unique<dai_nodes::Slam>("slam", node, pipeline, device, rsCompat, *stereo->getLeftSensor(), *vio, *stereo);
            //     } else if(stereo->getSocketID() == stereo->getRightSensor()->getSocketID()) {
            //         slam = std::make_unique<dai_nodes::Slam>("slam", node, pipeline, device, rsCompat, *stereo->getRightSensor(), *vio, *stereo);
            //     } else {
            //         throw std::runtime_error("Stereo socket is not left or right. Cannot create SLAM node.");
            //     }
            //
            //     daiNodes.push_back(std::move(slam));
            // }
            daiNodes.push_back(std::move(vio));
        }
        daiNodes.push_back(std::move(imu));
    }
    daiNodes.push_back(std::move(stereo));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> CamArray::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                           std::shared_ptr<dai::Device> device,
                                                                           std::shared_ptr<dai::Pipeline> pipeline,
                                                                           std::shared_ptr<param_handlers::PipelineGenParamHandler> ph,
                                                                           const std::string& deviceName,
                                                                           bool rsCompat,
                                                                           const std::string& /*nnType*/) {
    using namespace dai_nodes::sensor_helpers;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;

    for(auto& feature : device->getConnectedCameraFeatures()) {
        auto name = depthai_bridge::getSocketName(feature.socket, deviceName, rsCompat, true);
        auto daiNode = std::make_unique<dai_nodes::SensorWrapper>(name, node, pipeline, deviceName, rsCompat, feature.socket);
        daiNodes.push_back(std::move(daiNode));
    };
    if(checkForImu(ph, device, node->get_logger())) {
        auto imu = std::make_unique<dai_nodes::Imu>("imu", node, pipeline, device, rsCompat);
        daiNodes.push_back(std::move(imu));
    }
    return daiNodes;
}

std::vector<std::unique_ptr<dai_nodes::BaseNode>> DepthToF::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                           std::shared_ptr<dai::Device> device,
                                                                           std::shared_ptr<dai::Pipeline> pipeline,
                                                                           std::shared_ptr<param_handlers::PipelineGenParamHandler> ph,
                                                                           const std::string& deviceName,
                                                                           bool rsCompat,
                                                                           const std::string& /*nnType*/) {
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto tof = std::make_unique<dai_nodes::ToF>("tof", node, pipeline, deviceName, rsCompat);
    auto stereo = std::make_unique<dai_nodes::Stereo>("stereo", node, pipeline, device, rsCompat);
    if(stereo->isAligned() && tof->isAligned() && stereo->getSocketID() == tof->getAlignedSocketID()) {
        throw std::runtime_error(
            "Both ToF and Stereo alignment are enabled. Please disable alignment for one of them using the 'i_aligned' parameter for proper pipeline "
            "creation.");
    }
    // check if stereo is aligned to tof or vice-versa
    if(stereo->isAligned() && stereo->getSocketID() == tof->getAlignedSocketID()) {
        tof->link(stereo->getInput(static_cast<int>(dai_nodes::link_types::StereoLinkType::align)));
    } else if(tof->isAligned() && tof->getAlignedSocketID() == stereo->getSocketID()) {
        stereo->link(tof->getInput(), static_cast<int>(dai_nodes::link_types::StereoLinkType::stereo));
    }
    addRgbdNode(daiNodes, node, device, pipeline, ph, rsCompat, *stereo->getLeftSensor(), *tof);

    if(checkForImu(ph, device, node->get_logger())) {
        auto imu = std::make_unique<dai_nodes::Imu>("imu", node, pipeline, device, rsCompat);
        if(ph->getParam<bool>("i_enable_vio")) {
            auto vio = std::make_unique<dai_nodes::Vio>("vio", node, pipeline, device, rsCompat, *stereo, *imu);
            daiNodes.push_back(std::move(vio));
        }
        daiNodes.push_back(std::move(imu));
    }
    daiNodes.push_back(std::move(tof));
    daiNodes.push_back(std::move(stereo));
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> StereoToF::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                            std::shared_ptr<dai::Device> device,
                                                                            std::shared_ptr<dai::Pipeline> pipeline,
                                                                            std::shared_ptr<param_handlers::PipelineGenParamHandler> ph,
                                                                            const std::string& deviceName,
                                                                            bool rsCompat,
                                                                            const std::string& /*nnType*/) {
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto tof = std::make_unique<dai_nodes::ToF>("tof", node, pipeline, deviceName, rsCompat);
    auto left = std::make_unique<dai_nodes::SensorWrapper>("left", node, pipeline, deviceName, rsCompat, dai::CameraBoardSocket::CAM_B);
    auto right = std::make_unique<dai_nodes::SensorWrapper>("right", node, pipeline, deviceName, rsCompat, dai::CameraBoardSocket::CAM_C);
    if(tof->isAligned() && tof->getAlignedSocketID() == right->getSocketID()) {
        right->getDefaultOut()->link(tof->getInput());
    } else if(tof->isAligned() && tof->getAlignedSocketID() == left->getSocketID()) {
        left->getDefaultOut()->link(tof->getInput());
    }
    if(checkForImu(ph, device, node->get_logger())) {
        auto imu = std::make_unique<dai_nodes::Imu>("imu", node, pipeline, device, rsCompat);
        if(ph->getParam<bool>("i_enable_vio")) {
            auto vio = std::make_unique<dai_nodes::Vio>("vio", node, pipeline, device, rsCompat, *left, *right, *imu);
            daiNodes.push_back(std::move(vio));
        }
        daiNodes.push_back(std::move(imu));
    }
    daiNodes.push_back(std::move(left));
    daiNodes.push_back(std::move(right));
    daiNodes.push_back(std::move(tof));
    return daiNodes;
}

std::vector<std::unique_ptr<dai_nodes::BaseNode>> ToF::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                      std::shared_ptr<dai::Device> device,
                                                                      std::shared_ptr<dai::Pipeline> pipeline,
                                                                      std::shared_ptr<param_handlers::PipelineGenParamHandler> ph,
                                                                      const std::string& deviceName,
                                                                      bool rsCompat,
                                                                      const std::string& /*nnType*/) {
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto tof = std::make_unique<dai_nodes::ToF>("tof", node, pipeline, deviceName, rsCompat);
    daiNodes.push_back(std::move(tof));
    if(checkForImu(ph, device, node->get_logger())) {
        auto imu = std::make_unique<dai_nodes::Imu>("imu", node, pipeline, device, rsCompat);
        daiNodes.push_back(std::move(imu));
    }
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> RGBToF::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                         std::shared_ptr<dai::Device> device,
                                                                         std::shared_ptr<dai::Pipeline> pipeline,
                                                                         std::shared_ptr<param_handlers::PipelineGenParamHandler> ph,
                                                                         const std::string& deviceName,
                                                                         bool rsCompat,
                                                                         const std::string& nnType) {
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto rgb = std::make_unique<dai_nodes::SensorWrapper>("left", node, pipeline, deviceName, rsCompat, dai::CameraBoardSocket::CAM_B);
    auto tof = std::make_unique<dai_nodes::ToF>("tof", node, pipeline, deviceName, rsCompat);
    addNnNode(daiNodes, node, pipeline, deviceName, rsCompat, *rgb, nnType);
    if(tof->isAligned() && tof->getAlignedSocketID() == rgb->getSocketID()) {
        rgb->getDefaultOut()->link(tof->getInput());
    }
    addRgbdNode(daiNodes, node, device, pipeline, ph, rsCompat, *rgb, *tof);
    daiNodes.push_back(std::move(rgb));
    daiNodes.push_back(std::move(tof));
    if(checkForImu(ph, device, node->get_logger())) {
        auto imu = std::make_unique<dai_nodes::Imu>("imu", node, pipeline, device, rsCompat);
        daiNodes.push_back(std::move(imu));
    }
    return daiNodes;
}
std::vector<std::unique_ptr<dai_nodes::BaseNode>> Thermal::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                          std::shared_ptr<dai::Device> device,
                                                                          std::shared_ptr<dai::Pipeline> pipeline,
                                                                          std::shared_ptr<param_handlers::PipelineGenParamHandler> ph,
                                                                          const std::string& deviceName,
                                                                          bool rsCompat,
                                                                          const std::string& nnType) {
    using namespace dai_nodes::sensor_helpers;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    auto rgb = std::make_unique<dai_nodes::SensorWrapper>("rgb", node, pipeline, deviceName, rsCompat, dai::CameraBoardSocket::CAM_A);
    addNnNode(daiNodes, node, pipeline, deviceName, rsCompat, *rgb, nnType);
    auto thermal = std::make_unique<dai_nodes::Thermal>("thermal", node, pipeline, deviceName, rsCompat);
    daiNodes.push_back(std::move(rgb));
    daiNodes.push_back(std::move(thermal));
    if(checkForImu(ph, device, node->get_logger())) {
        auto imu = std::make_unique<dai_nodes::Imu>("imu", node, pipeline, device, rsCompat);
        daiNodes.push_back(std::move(imu));
    }
    return daiNodes;
}
}  // namespace pipeline_gen
}  // namespace depthai_ros_driver

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::RGB, depthai_ros_driver::pipeline_gen::BasePipeline)
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::RGBD, depthai_ros_driver::pipeline_gen::BasePipeline)
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::RGBStereo, depthai_ros_driver::pipeline_gen::BasePipeline)
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::Stereo, depthai_ros_driver::pipeline_gen::BasePipeline)
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::Depth, depthai_ros_driver::pipeline_gen::BasePipeline)
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::CamArray, depthai_ros_driver::pipeline_gen::BasePipeline)
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::StereoToF, depthai_ros_driver::pipeline_gen::BasePipeline)
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::DepthToF, depthai_ros_driver::pipeline_gen::BasePipeline)
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::ToF, depthai_ros_driver::pipeline_gen::BasePipeline)
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::RGBToF, depthai_ros_driver::pipeline_gen::BasePipeline)
PLUGINLIB_EXPORT_CLASS(depthai_ros_driver::pipeline_gen::Thermal, depthai_ros_driver::pipeline_gen::BasePipeline)
