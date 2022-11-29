#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"

#include <fstream>
#include <nlohmann/json.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/nodes.hpp"

namespace depthai_ros_driver {
namespace param_handlers {

NNParamHandler::NNParamHandler(const std::string& name) : BaseParamHandler(name){};

void NNParamHandler::parseConfigFile(rclcpp::Node* node, const std::string& path, std::shared_ptr<dai::node::SpatialDetectionNetwork> nn, std::shared_ptr<dai::node::ImageManip> image_manip) {
    using json = nlohmann::json;
    std::ifstream f(path);
    json data = json::parse(f);
    if(data.contains("model") && data.contains("nn_config")) {
        auto nn_family = data["nn_config"]["NN_family"];
        auto source = data["model"]["zoo"].get<std::string>();
        if(source == "depthai_ros_driver") {
            std::string model_path =
                ament_index_cpp::get_package_share_directory("depthai_ros_driver") + "/models/" + data["model"]["model_name"].get<std::string>() + ".blob";
            auto blob = dai::OpenVINO::Blob(model_path);
            auto first_info = blob.networkInputs.begin();
            RCLCPP_INFO(node->get_logger(), "Input size %d %d",first_info->second.dims[0], first_info->second.dims[1]);
            auto input_size = declareAndLogParam<int>(node,"i_input_size", first_info->second.dims[0]);
            // image_manip->setMaxOutputFrameSize(input_size*input_size*3);
            // image_manip->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);
            image_manip->inputImage.setBlocking(false);
            image_manip->inputImage.setQueueSize(2);
            image_manip->initialConfig.setResize(input_size, input_size);
            nn->setBlobPath(model_path);
            nn->setNumPoolFrames(4);
            nn->setNumInferenceThreads(declareAndLogParam<int>(node, "i_num_inference_threads", 2));
            nn->input.setBlocking(false);
        }
    }
}

void NNParamHandler::parseConfigFile(rclcpp::Node* node, const std::string& path, std::shared_ptr<dai::node::NeuralNetwork> nn, std::shared_ptr<dai::node::ImageManip> image_manip) {
    using json = nlohmann::json;
    std::ifstream f(path);
    json data = json::parse(f);
    if(data.contains("model") && data.contains("nn_config")) {
        auto nn_family = data["nn_config"]["NN_family"];
        auto source = data["model"]["zoo"].get<std::string>();
        if(source == "depthai_ros_driver") {
            std::string model_path =
                ament_index_cpp::get_package_share_directory("depthai_ros_driver") + "/models/" + data["model"]["model_name"].get<std::string>() + ".blob";
            auto blob = dai::OpenVINO::Blob(model_path);
            auto first_info = blob.networkInputs.begin();
            RCLCPP_INFO(node->get_logger(), "Input size %d %d",first_info->second.dims[0], first_info->second.dims[1]);
            auto input_size = declareAndLogParam<int>(node,"i_input_size", first_info->second.dims[0]);
            // image_manip->setMaxOutputFrameSize(input_size*input_size*3);
            // image_manip->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);
            image_manip->inputImage.setBlocking(false);
            image_manip->inputImage.setQueueSize(2);
            image_manip->initialConfig.setResize(input_size, input_size);
            nn->setBlobPath(model_path);
            nn->setNumPoolFrames(4);
            nn->setNumInferenceThreads(declareAndLogParam<int>(node, "i_num_inference_threads", 2));
            nn->input.setBlocking(false);
        }
    }
}

void NNParamHandler::declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::NeuralNetwork> nn, std::shared_ptr<dai::node::ImageManip> image_manip) {
    declareAndLogParam<int>(node, "i_max_q_size", 4);
    std::string default_nn_path = ament_index_cpp::get_package_share_directory("depthai_ros_driver") + "/config/nn/segmentation.json";
    auto nn_path = declareAndLogParam<std::string>(node, "i_nn_path", default_nn_path);
    parseConfigFile(node, nn_path, nn, image_manip);

}
void NNParamHandler::declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::SpatialDetectionNetwork> nn, std::shared_ptr<dai::node::ImageManip> image_manip) {
    declareAndLogParam<int>(node, "i_max_q_size", 4);
    std::string default_nn_path = ament_index_cpp::get_package_share_directory("depthai_ros_driver") + "/config/nn/mobilenet.json";
    auto nn_path = declareAndLogParam<std::string>(node, "i_nn_path", default_nn_path);
    parseConfigFile(node, nn_path, nn, image_manip);
}
dai::CameraControl NNParamHandler::setRuntimeParams(rclcpp::Node* node, const std::vector<rclcpp::Parameter>& params) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver