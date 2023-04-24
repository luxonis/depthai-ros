#include "depthai_ros_driver/utils.hpp"
namespace depthai_ros_driver {
namespace utils {
std::string getUpperCaseStr(const std::string& string) {
    std::string upper = string;
    for(auto& c : upper) c = toupper(c);
    return upper;
}
}  // namespace utils
}  // namespace depthai_ros_driver