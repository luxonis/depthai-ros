#include "depthai_ros_driver/utils.hpp"

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/CameraFeatures.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
namespace depthai_ros_driver {
namespace utils {
std::string getUpperCaseStr(const std::string& string) {
    std::string upper = string;
    for(auto& c : upper) c = toupper(c);
    return upper;
}
std::string getSocketName(dai::CameraBoardSocket socket) {
    return dai_nodes::sensor_helpers::socketNameMap.at(socket);
}
}  // namespace utils
}  // namespace depthai_ros_driver