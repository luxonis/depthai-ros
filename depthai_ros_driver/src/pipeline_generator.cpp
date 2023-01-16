#include "depthai_ros_driver/pipeline_generator.hpp"

#include "depthai_ros_driver/dai_nodes/nn/nn_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/nn/nn_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/nn/spatial_nn_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/camera_sensor.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/imu.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/stereo.hpp"

namespace depthai_ros_driver {
namespace pipeline_gen {
std::vector<std::unique_ptr<dai_nodes::BaseNode>> PipelineGenerator::createPipeline(ros::NodeHandle node,
                                                                                    std::shared_ptr<dai::Device> device,
                                                                                    std::shared_ptr<dai::Pipeline> pipeline,
                                                                                    const std::string& pipelineType,
                                                                                    const std::string& nnType) {
    ROS_INFO("Pipeline type: %s", pipelineType.c_str());
    auto pType = pipelineTypeMap.at(pipelineType);
    auto nType = nnTypeMap.at(nnType);
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    switch(pType) {
        case PipelineType::RGB: {
            auto rgb = std::make_unique<dai_nodes::CameraSensor>("rgb", node, pipeline, device, dai::CameraBoardSocket::RGB);

            switch(nType) {
                case NNType::None:
                    break;
                case NNType::RGB: {
                    auto nn = std::make_unique<dai_nodes::NNWrapper>("nn", node, pipeline);
                    rgb->link(nn->getInput(), static_cast<int>(dai_nodes::link_types::RGBLinkType::preview));
                    daiNodes.push_back(std::move(nn));
                    break;
                }
                case NNType::Spatial: {
                    ROS_WARN("Spatial NN selected, but configuration is RGB.");
                }
                default:
                    break;
            }
            daiNodes.push_back(std::move(rgb));
            break;
        }

        case PipelineType::RGBD: {
            auto rgb = std::make_unique<dai_nodes::CameraSensor>("rgb", node, pipeline, device, dai::CameraBoardSocket::RGB);
            auto stereo = std::make_unique<dai_nodes::Stereo>("stereo", node, pipeline, device);
            switch(nType) {
                case NNType::None:
                    break;
                case NNType::RGB: {
                    auto nn = std::make_unique<dai_nodes::NNWrapper>("nn", node, pipeline);
                    rgb->link(nn->getInput(), static_cast<int>(dai_nodes::link_types::RGBLinkType::preview));
                    daiNodes.push_back(std::move(nn));
                    break;
                }
                case NNType::Spatial: {
                    auto nn = std::make_unique<dai_nodes::SpatialNNWrapper>("nn", node, pipeline);
                    rgb->link(nn->getInput(static_cast<int>(dai_nodes::nn_helpers::link_types::SpatialNNLinkType::input)),
                              static_cast<int>(dai_nodes::link_types::RGBLinkType::preview));
                    stereo->link(nn->getInput(static_cast<int>(dai_nodes::nn_helpers::link_types::SpatialNNLinkType::inputDepth)));
                    daiNodes.push_back(std::move(nn));
                    break;
                }
                default:
                    break;
            }
            daiNodes.push_back(std::move(rgb));
            daiNodes.push_back(std::move(stereo));
            break;
        }
        case PipelineType::RGBStereo: {
            auto rgb = std::make_unique<dai_nodes::CameraSensor>("rgb", node, pipeline, device, dai::CameraBoardSocket::RGB);
            auto left = std::make_unique<dai_nodes::CameraSensor>("left", node, pipeline, device, dai::CameraBoardSocket::LEFT);
            auto right = std::make_unique<dai_nodes::CameraSensor>("right", node, pipeline, device, dai::CameraBoardSocket::RIGHT);
            daiNodes.push_back(std::move(rgb));
            daiNodes.push_back(std::move(left));
            daiNodes.push_back(std::move(right));
            break;
        }
        case PipelineType::Stereo: {
            auto left = std::make_unique<dai_nodes::CameraSensor>("left", node, pipeline, device, dai::CameraBoardSocket::LEFT);
            auto right = std::make_unique<dai_nodes::CameraSensor>("right", node, pipeline, device, dai::CameraBoardSocket::RIGHT);
            daiNodes.push_back(std::move(left));
            daiNodes.push_back(std::move(right));
            break;
        }
        default: {
            std::string configuration = pipelineType;
            throw std::runtime_error("UNKNOWN PIPELINE TYPE SPECIFIED/CAMERA DOESN'T SUPPORT GIVEN PIPELINE. Configuration: " + configuration);
        }
    }
    auto imu = std::make_unique<dai_nodes::Imu>("imu", node, pipeline);
    daiNodes.push_back(std::move(imu));

    ROS_INFO("Finished setting up pipeline.");
    return daiNodes;
}
}  // namespace pipeline_gen
}  // namespace depthai_ros_driver