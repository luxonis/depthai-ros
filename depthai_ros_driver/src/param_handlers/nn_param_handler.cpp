#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"

#include <fstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {

NNParamHandler::NNParamHandler(rclcpp::Node* node, const std::string& name) : BaseParamHandler(node, name) {
    nnFamilyMap = {
        {"segmentation", nn::NNFamily::Segmentation},
        {"mobilenet", nn::NNFamily::Mobilenet},
        {"YOLO", nn::NNFamily::Yolo},
    };
}
NNParamHandler::~NNParamHandler() = default;
nn::NNFamily NNParamHandler::getNNFamily() {
    std::string config_path = ament_index_cpp::get_package_share_directory("depthai_ros_driver") + "/config/nn/";
    std::string default_nn_conf_name = "mobilenet.json";
    std::string default_path = config_path + default_nn_conf_name;
    auto nn_path = declareAndLogParam<std::string>("i_nn_config_path", default_path);
    if(nn_path == "depthai_ros_driver/yolo") {
        nn_path = config_path + "yolo.json";
    } else if(nn_path == "depthai_ros_driver/segmentation") {
        nn_path = config_path + "segmentation.json";
    } else if(nn_path == "depthai_ros_driver/mobilenet") {
        nn_path = config_path + "mobilenet.json";
    }
    auto final_path = declareAndLogParam<std::string>("i_nn_config_path", nn_path, true);
    using json = nlohmann::json;
    std::ifstream f(final_path);
    json data = json::parse(f);
    std::string nnFamily;
    if(data.contains("model") && data.contains("nn_config")) {
        nnFamily = data["nn_config"]["NN_family"].get<std::string>();
        RCLCPP_INFO(getROSNode()->get_logger(), "NN Family: %s", nnFamily.c_str());
    } else {
        throw std::runtime_error("No required fields");
    }
    return utils::getValFromMap(nnFamily, nnFamilyMap);
}

void NNParamHandler::setNNParams(nlohmann::json data, std::shared_ptr<dai::node::NeuralNetwork> /*nn*/) {
    auto labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    if(!labels.empty()) {
        declareAndLogParam<std::vector<std::string>>("i_label_map", labels);
    }
}

void NNParamHandler::setNNParams(nlohmann::json data, std::shared_ptr<dai::node::MobileNetDetectionNetwork> nn) {
    if(data["nn_config"].contains("confidence_threshold")) {
        auto conf_threshold = data["nn_config"]["confidence_threshold"].get<float>();
        nn->setConfidenceThreshold(conf_threshold);
    }
    auto labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    if(!labels.empty()) {
        declareAndLogParam<std::vector<std::string>>("i_label_map", labels);
    }
}

void NNParamHandler::setNNParams(nlohmann::json data, std::shared_ptr<dai::node::MobileNetSpatialDetectionNetwork> nn) {
    if(data["nn_config"].contains("confidence_threshold")) {
        auto conf_threshold = data["nn_config"]["confidence_threshold"].get<float>();
        nn->setConfidenceThreshold(conf_threshold);
    }
    auto labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    if(!labels.empty()) {
        declareAndLogParam<std::vector<std::string>>("i_label_map", labels);
    }
    setSpatialParams(nn);
}
void NNParamHandler::setNNParams(nlohmann::json data, std::shared_ptr<dai::node::YoloSpatialDetectionNetwork> nn) {
    float conf_threshold = 0.5;
    if(data["nn_config"].contains("confidence_threshold")) {
        conf_threshold = data["nn_config"]["confidence_threshold"].get<float>();
        nn->setConfidenceThreshold(conf_threshold);
    }
    auto labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    if(!labels.empty()) {
        declareAndLogParam<std::vector<std::string>>("i_label_map", labels);
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
    auto labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    if(!labels.empty()) {
        declareAndLogParam<std::vector<std::string>>("i_label_map", labels);
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
    RCLCPP_INFO(getROSNode()->get_logger(), "NN input size: %d x %d. Resizing input image in case of different dimensions.", inputSize, inputSize);
    imageManip->initialConfig.setResize(inputSize, inputSize);
}
std::string NNParamHandler::getModelPath(const nlohmann::json& data) {
    std::string modelPath;
    auto source = data["model"]["zoo"].get<std::string>();
    if(source == "depthai_examples") {
        modelPath = ament_index_cpp::get_package_share_directory("depthai_examples") + "/resources/" + data["model"]["model_name"].get<std::string>() + ".blob";
    } else if(source == "path") {
        modelPath = data["model"]["model_name"].get<std::string>();
    } else {
        throw std::runtime_error("Other options not yet available");
    }
    return modelPath;
}

dai::CameraControl NNParamHandler::setRuntimeParams(const std::vector<rclcpp::Parameter>& /*params*/) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver