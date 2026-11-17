#include "depthai_ros_driver_v3/utils.hpp"

#include <stdexcept>

#include "depthai/pipeline/Pipeline.hpp"
namespace depthai_ros_driver {
namespace utils {
std::string getUpperCaseStr(const std::string& string) {
    std::string upper = string;
    for(auto& c : upper) c = toupper(c);
    return upper;
}

dai::PipelineAutoCalibrationMode parsePipelineAutoCalibrationMode(const std::string& mode) {
    const auto normalizedMode = getUpperCaseStr(mode);
    if(normalizedMode == "OFF") {
        return dai::PipelineAutoCalibrationMode::OFF;
    }
    if(normalizedMode == "ON_START") {
        return dai::PipelineAutoCalibrationMode::ON_START;
    }
    if(normalizedMode == "CONTINUOUS") {
        return dai::PipelineAutoCalibrationMode::CONTINUOUS;
    }
    throw std::invalid_argument("Invalid pipeline auto calibration mode '" + mode + "'. Valid values are OFF, ON_START, CONTINUOUS, or empty.");
}
}  // namespace utils
}  // namespace depthai_ros_driver
