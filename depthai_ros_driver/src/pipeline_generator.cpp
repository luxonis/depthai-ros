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
                                                                                    const std::string& nnType,
                                                                                    bool enableImu) {
    ROS_INFO("Pipeline type: %s", pipelineType.c_str());
    std::string pTypeUpCase = pipelineType;
    std::string nTypeUpCase = nnType;
    for(auto& c : pTypeUpCase) c = toupper(c);
    for(auto& c : nTypeUpCase) c = toupper(c);
    auto pType = pipelineTypeMap.at(pTypeUpCase);
    pType = validatePipeline(pType, device->getCameraSensorNames().size());
    auto nType = nnTypeMap.at(nTypeUpCase);
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    switch(pType) {
        case PipelineType::RGB: {
            auto rgb = std::make_unique<dai_nodes::CameraSensor>("rgb", node, pipeline, device, dai::CameraBoardSocket::RGB);
            switch(nType) {
                case NNType::None:
                    break;
                case NNType::RGB: {
                    auto nn = createNN(node, pipeline, *rgb);
                    daiNodes.push_back(std::move(nn));
                    break;
                }
                case NNType::Spatial: {
                    ROS_WARN("Spatial NN selected, but configuration is RGB. NN not created.");
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
                    auto nn = createNN(node, pipeline, *rgb);
                    daiNodes.push_back(std::move(nn));
                    break;
                }
                case NNType::Spatial: {
                    auto nn = createSpatialNN(node, pipeline, *rgb, *stereo);
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
            switch(nType) {
                case NNType::None:
                    break;
                case NNType::RGB: {
                    auto nn = createNN(node, pipeline, *rgb);
                    daiNodes.push_back(std::move(nn));
                    break;
                }
                case NNType::Spatial: {
                    ROS_WARN("Spatial NN selected, but configuration is RGBStereo. NN not created.");
                }
                default:
                    break;
            }
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
        case PipelineType::Depth: {
            auto stereo = std::make_unique<dai_nodes::Stereo>("stereo", node, pipeline, device);
            daiNodes.push_back(std::move(stereo));
            break;
        }
        case PipelineType::CamArray: {
            int i = 0;
            int j = 0;
            for(auto& sensor : device->getCameraSensorNames()) {
                // append letter for greater sensor number
                if(i % alphabet.size() == 0) {
                    j++;
                }
                std::string nodeName(j, alphabet[i % alphabet.size()]);
                auto daiNode = std::make_unique<dai_nodes::CameraSensor>(nodeName, node, pipeline, device, sensor.first);
                daiNodes.push_back(std::move(daiNode));
                i++;
            };
            break;
        }
        default: {
            std::string configuration = pipelineType;
            throw std::runtime_error("UNKNOWN PIPELINE TYPE SPECIFIED/CAMERA DOESN'T SUPPORT GIVEN PIPELINE. Configuration: " + configuration);
        }
    }
    if(enableImu) {
        auto imu = std::make_unique<dai_nodes::Imu>("imu", node, pipeline);
        daiNodes.push_back(std::move(imu));
    }
    ROS_INFO("Finished setting up pipeline.");
    return daiNodes;
}
std::unique_ptr<dai_nodes::BaseNode> PipelineGenerator::createNN(ros::NodeHandle node, std::shared_ptr<dai::Pipeline> pipeline, dai_nodes::BaseNode& daiNode) {
    auto nn = std::make_unique<dai_nodes::NNWrapper>("nn", node, pipeline);
    daiNode.link(nn->getInput(), static_cast<int>(dai_nodes::link_types::RGBLinkType::preview));
    return nn;
}
std::unique_ptr<dai_nodes::BaseNode> PipelineGenerator::createSpatialNN(ros::NodeHandle node,
                                                                        std::shared_ptr<dai::Pipeline> pipeline,
                                                                        dai_nodes::BaseNode& daiNode,
                                                                        dai_nodes::BaseNode& daiStereoNode) {
    auto nn = std::make_unique<dai_nodes::SpatialNNWrapper>("nn", node, pipeline);
    daiNode.link(nn->getInput(static_cast<int>(dai_nodes::nn_helpers::link_types::SpatialNNLinkType::input)),
                 static_cast<int>(dai_nodes::link_types::RGBLinkType::preview));
    daiStereoNode.link(nn->getInput(static_cast<int>(dai_nodes::nn_helpers::link_types::SpatialNNLinkType::inputDepth)));
    return nn;
}
PipelineType PipelineGenerator::validatePipeline(PipelineType type, int sensorNum) {
    if(sensorNum == 1) {
        if(type != PipelineType::RGB) {
            ROS_ERROR("Wrong pipeline chosen for camera as it has only one sensor. Switching to RGB.");
            return PipelineType::RGB;
        }
    } else if(sensorNum == 2) {
        if(type != PipelineType::Stereo || type != PipelineType::Depth) {
            ROS_ERROR("Wrong pipeline chosen for camera as it has only stereo pair. Switching to Stereo.");
            return PipelineType::Stereo;
        }
    } else if(sensorNum > 3 && type != PipelineType::CamArray) {
        ROS_ERROR("For cameras with more than three sensors you can only use CamArray. Switching to CamArray.");
        return PipelineType::CamArray;
    }
    return type;
}

}  // namespace pipeline_gen
}  // namespace depthai_ros_driver