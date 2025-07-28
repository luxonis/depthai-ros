#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"


#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {

NNParamHandler::NNParamHandler(
    std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat, const dai::CameraBoardSocket& socket)
    : BaseParamHandler(node, name, deviceName, rsCompat) {
    nnFamilyMap = {
        {"segmentation", nn::NNFamily::Segmentation},
        {"detection", nn::NNFamily::Detection},
    };
    declareAndLogParam<int>("i_board_socket_id", static_cast<int>(socket));
}
NNParamHandler::~NNParamHandler() = default;
nn::NNFamily NNParamHandler::getNNFamily() {
    std::string nnFamily = declareAndLogParam<std::string>("i_nn_family", "detection");
    std::string nnModel = declareAndLogParam<std::string>("i_nn_model", "yolov6-nano");
    return utils::getValFromMap(nnFamily, nnFamilyMap);
}

void NNParamHandler::setNNParams(std::shared_ptr<dai::node::NeuralNetwork> /*nn*/) {
    // auto labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    // if(!labels.empty()) {
    //     declareAndLogParam<std::vector<std::string>>("i_label_map", labels);
    // }
}

void NNParamHandler::setNNParams(std::shared_ptr<dai::node::DetectionNetwork> nn) {
    nn->setConfidenceThreshold(declareAndLogParam<float>("i_nn_confidence_threshold", 0.5));
    // auto labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    // if(!labels.empty()) {
    //     declareAndLogParam<std::vector<std::string>>("i_label_map", labels);
    // }
}

void NNParamHandler::setNNParams(std::shared_ptr<dai::node::SpatialDetectionNetwork> nn) {

    nn->setConfidenceThreshold(declareAndLogParam<float>("i_nn_confidence_threshold", 0.5));
    // auto labels = data["mappings"]["labels"].get<std::vector<std::string>>();
    // if(!labels.empty()) {
    //     declareAndLogParam<std::vector<std::string>>("i_label_map", labels);
    // }
    setSpatialParams(nn);
}


}  // namespace param_handlers
}  // namespace depthai_rosdeclareAndLogParam<std::string>("i_nn_family", "detection");_driver
