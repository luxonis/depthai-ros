#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "depthai/depthai.hpp"
#include "depthai/pipeline/nodes.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
NNParamHandler::NNParamHandler(const std::string& name) : BaseParamHandler(name){};
void NNParamHandler::declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::NeuralNetwork> nn) {
    declareAndLogParam<int>(node, "i_max_q_size", 4);
    auto config_path = node->get_parameter("i_nn_config_path").as_string();
    std::string default_nn_path = ament_index_cpp::get_package_share_directory("depthai_ros_driver") +
    "/models/deeplab_v3_plus_mnv2_decoder_256_openvino_2021.4.blob";
    auto nn_path = declareAndLogParam<std::string>(node, "i_nn_path", default_nn_path);
    nn->setBlobPath(nn_path);
    nn->setNumPoolFrames(4);
    nn->setNumInferenceThreads(declareAndLogParam<int>(node, "i_num_inference_threads", 2));
    nn->input.setBlocking(declareAndLogParam<bool>(node, "i_set_blocking", false));

}
dai::CameraControl NNParamHandler::setRuntimeParams(rclcpp::Node* node,const std::vector<rclcpp::Parameter>& params) {
    dai::CameraControl ctrl;
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver