#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"

#include <fstream>
#include <nlohmann/json.hpp>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/nodes.hpp"
#include "ros/package.h"

namespace depthai_ros_driver {
namespace param_handlers {

NNParamHandler::NNParamHandler(const std::string& name) : BaseParamHandler(name) {
    nnFamilyMap = {
        {"segmentation", nn::NNFamily::Segmentation},
        {"mobilenet", nn::NNFamily::Mobilenet},
        {"YOLO", nn::NNFamily::Yolo},
    };
}
NNParamHandler::~NNParamHandler() = default;
std::string NNParamHandler::getConfigPath(ros::NodeHandle node) {
    std::string configPath = ros::package::getPath("depthai_ros_driver") + "/config/nn/";
    auto nnPath = getParam<std::string>(node, "i_nn_config_path");
    if(nnPath == "depthai_ros_driver/yolo") {
        nnPath = configPath + "yolo.json";
    } else if(nnPath == "depthai_ros_driver/segmentation") {
        nnPath = configPath + "segmentation.json";
    } else if(nnPath == "depthai_ros_driver/mobilenet") {
        nnPath = configPath + "mobilenet.json";
    }
    return nnPath;
}
nn::NNFamily NNParamHandler::getNNFamily(ros::NodeHandle node) {
    auto nnPath = getConfigPath(node);
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
    auto first_info = blob.networkInputs.begin();
    auto input_size = first_info->second.dims[0];

    imageManip->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);
    imageManip->inputImage.setBlocking(false);
    imageManip->inputImage.setQueueSize(8);
    imageManip->initialConfig.setResize(input_size, input_size);
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

dai::CameraControl NNParamHandler::setRuntimeParams(ros::NodeHandle /*node*/, parametersConfig& /*config*/) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver