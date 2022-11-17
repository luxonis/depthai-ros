#include "depthai_ros_driver/param_handlers/rgb_param_handler.hpp"

#include "depthai/include/depthai-shared/properties/ColorCameraProperties.hpp"
#include "depthai/pipeline/nodes.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
RGBParamHandler::RGBParamHandler(const std::string &dai_node_name): BaseParamHandler(dai_node_name){
};
void RGBParamHandler::declareParams(rclcpp::Node* node, std::shared_ptr<dai::node::ColorCamera> color_cam) {
    color_cam->setFps(declareAndLogParam<double>(node, "i_fps", 30.0));
    color_cam->setResolution(rgb_resolution_map_.at(declareAndLogParam<std::string>(node, "i_resolution", "1080")));
    color_cam->setInterleaved(declareAndLogParam<bool>(node, "i_interleaved", false));
    if(declareAndLogParam<bool>(node, "i_set_isp", false)){
        color_cam->setIspScale(2,3);
    }
    color_cam->setPreviewKeepAspectRatio(declareAndLogParam(node, "i_keep_preview_aspect_ratio", true));

}
}  // namespace param_handlers
}  // namespace depthai_ros_driver