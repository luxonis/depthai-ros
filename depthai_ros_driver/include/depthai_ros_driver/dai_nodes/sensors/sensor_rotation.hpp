#pragma once

#include <optional>

#include "depthai/common/CameraImageOrientation.hpp"

namespace depthai_ros_driver::dai_nodes::sensor_helpers {

// Compose with the board's native readout orientation, rather than assuming
// AUTO means NORMAL. A half turn toggles both readout axes.
inline std::optional<dai::CameraImageOrientation> halfTurnOrientation(dai::CameraImageOrientation native) {
    using Orientation = dai::CameraImageOrientation;
    switch(native) {
        case Orientation::NORMAL:
            return Orientation::ROTATE_180_DEG;
        case Orientation::ROTATE_180_DEG:
            return Orientation::NORMAL;
        case Orientation::HORIZONTAL_MIRROR:
            return Orientation::VERTICAL_FLIP;
        case Orientation::VERTICAL_FLIP:
            return Orientation::HORIZONTAL_MIRROR;
        default:
            return std::nullopt;
    }
}

}  // namespace depthai_ros_driver::dai_nodes::sensor_helpers
