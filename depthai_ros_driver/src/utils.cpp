#include "depthai_ros_driver/utils.hpp"

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/CameraFeatures.hpp"
namespace depthai_ros_driver {
namespace utils {
std::string getUpperCaseStr(const std::string& string) {
    std::string upper = string;
    for(auto& c : upper) c = toupper(c);
    return upper;
}
std::string getSocketName(dai::CameraBoardSocket socket, std::vector<dai::CameraFeatures> camFeatures) {
    std::string name;
    for(auto& cam : camFeatures) {
        if(cam.socket == socket) {
            if(cam.name == "color" || cam.name == "center") {
                name = "rgb";
            } else {
                name = cam.name;
            }
            return name;
        }
    }
    throw std::runtime_error("Camera socket not found");
}
}  // namespace utils
}  // namespace depthai_ros_driver