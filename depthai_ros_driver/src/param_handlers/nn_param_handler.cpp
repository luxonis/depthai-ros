#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"

#include <fstream>

#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "nlohmann/json.hpp"
#include "ros/node_handle.h"
#include "ros/package.h"

namespace depthai_ros_driver {
namespace param_handlers {

NNParamHandler::NNParamHandler(ros::NodeHandle node, const std::string& name) : BaseParamHandler(node, name) {
    nnFamilyMap = {
        {"segmentation", nn::NNFamily::Segmentation},
        {"mobilenet", nn::NNFamily::Mobilenet},
        {"YOLO", nn::NNFamily::Yolo},
    };
}
NNParamHandler::~NNParamHandler() = default;
std::string NNParamHandler::getConfigPath() {
    std::string configPath = ros::package::getPath("depthai_ros_driver") + "/config/nn/";
    auto nnPath = getParam<std::string>("i_nn_config_path");
    if(nnPath == "depthai_ros_driver/yolo") {
        nnPath = configPath + "yolo.json";
    } else if(nnPath == "depthai_ros_driver/segmentation") {
        nnPath = configPath + "segmentation.json";
    } else if(nnPath == "depthai_ros_driver/mobilenet") {
        nnPath = configPath + "mobilenet.json";
    }
    return nnPath;
}
nn::NNFamily NNParamHandler::getNNFamily() {
    auto nnPath = getConfigPath();
    using json = nlohmann::json;
    std::ifstream f(nnPath);
    json data = json::parse(f);
    std::string nnFamily;
    if(data.contains("model") && data.contains("nn_config")) {
        nnFamily = data["nn_config"]["NN_family"].get<std::string>();
        ROS_INFO("NN Family: %s", nnFamily.c_str());
    } else {
        throw std::runtime_error("No required fields");
    }
    return nnFamilyMap.at(nnFamily);
}

void NNParamHandler::setNNParams(nlohmann::json data, std::shared_ptr<dai::node::NeuralNetwork> /*nn*/) {
    if(data["mappings"].contains("labels")) {
        labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    }
}

void NNParamHandler::setNNParams(nlohmann::json data, std::shared_ptr<dai::node::MobileNetDetectionNetwork> nn) {
    if(data["nn_config"].contains("confidence_threshold")) {
        auto conf_threshold = data["nn_config"]["confidence_threshold"].get<float>();
        nn->setConfidenceThreshold(conf_threshold);
    }
    if(data["mappings"].contains("labels")) {
        labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    }
}

void NNParamHandler::setNNParams(nlohmann::json data, std::shared_ptr<dai::node::MobileNetSpatialDetectionNetwork> nn) {
    if(data["nn_config"].contains("confidence_threshold")) {
        auto conf_threshold = data["nn_config"]["confidence_threshold"].get<float>();
        nn->setConfidenceThreshold(conf_threshold);
    }
    if(data["mappings"].contains("labels")) {
        labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    }
    setSpatialParams(nn);
}
void NNParamHandler::setNNParams(nlohmann::json data, std::shared_ptr<dai::node::YoloSpatialDetectionNetwork> nn) {
    float conf_threshold = 0.5;
    if(data["nn_config"].contains("confidence_threshold")) {
        conf_threshold = data["nn_config"]["confidence_threshold"].get<float>();
        nn->setConfidenceThreshold(conf_threshold);
    }
    if(data["mappings"].contains("labels")) {
        labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    }
    setSpatialParams(nn);
    if(data["nn_config"].contains("NN_specific_metadata")) {
        setYoloParams(data, nn);
    }
}

void NNParamHandler::setNNParams(nlohmann::json data, std::shared_ptr<dai::node::YoloDetectionNetwork> nn) {
    float conf_threshold = 0.5;
    if(data["nn_config"].contains("confidence_threshold")) {
        conf_threshold = data["nn_config"]["confidence_threshold"].get<float>();
        nn->setConfidenceThreshold(conf_threshold);
    }
    if(data["mappings"].contains("labels")) {
        labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    }
    if(data["nn_config"].contains("NN_specific_metadata")) {
        setYoloParams(data, nn);
    }
}

void NNParamHandler::setImageManip(const std::string& model_path, std::shared_ptr<dai::node::ImageManip> imageManip) {
    auto blob = dai::OpenVINO::Blob(model_path);
    auto firstInfo = blob.networkInputs.begin();
    auto inputSize = firstInfo->second.dims[0];

    if(inputSize > 590) {
        std::ostringstream stream;
        stream << "Current network input size is too large to resize. Please set following parameters: rgb.i_preview_size: " << inputSize;
        stream << " and nn.i_disable_resize to true";
        throw std::runtime_error(stream.str());
    }
    imageManip->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);
    imageManip->inputImage.setBlocking(false);
    imageManip->inputImage.setQueueSize(8);
    imageManip->setKeepAspectRatio(false);
    ROS_INFO("NN input size: %d x %d. Resizing input image in case of different dimensions.", inputSize, inputSize);
    imageManip->initialConfig.setResize(inputSize, inputSize);
}
std::string NNParamHandler::getModelPath(const nlohmann::json& data) {
    std::string modelPath;
    auto source = data["model"]["zoo"].get<std::string>();
    if(source == "depthai_examples") {
        modelPath = ros::package::getPath("depthai_examples") + "/resources/" + data["model"]["model_name"].get<std::string>() + ".blob";
    } else if(source == "path") {
        modelPath = data["model"]["model_name"].get<std::string>();
    } else {
        throw std::runtime_error("Other options not yet available");
    }
    return modelPath;
}

dai::CameraControl NNParamHandler::setRuntimeParams(parametersConfig& /*config*/) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver