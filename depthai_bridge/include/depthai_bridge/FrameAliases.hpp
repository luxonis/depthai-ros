#pragma once

#include <string>
#include <utility>
#include <vector>

#include "depthai_bridge/depthaiUtility.hpp"

namespace depthai_bridge::detail {
// DEPRECATED frame spellings, retained as identity children during migration.
// The caller must omit aliases whose names already belong to actual camera frames.
inline std::vector<std::pair<std::string, std::string>> legacyCameraFrameAliases(const std::string& prefix,
                                                                                 dai::CameraBoardSocket socket,
                                                                                 const std::string& model,
                                                                                 bool rs) {
    const auto name = getSocketName(socket, model, rs);
    const auto body = getFrameName(prefix, name + (rs ? "_frame" : "_camera_frame"));
    const auto optical = getOpticalFrameName(prefix, name, rs);
    std::vector<std::pair<std::string, std::string>> result;
    if(rs) result.emplace_back(body, getFrameName(prefix, name + "_camera_frame"));
    std::string oldName;
    switch(socket) {
        case dai::CameraBoardSocket::CAM_A:
            oldName = rs ? "color" : "rgb";
            break;
        case dai::CameraBoardSocket::CAM_B:
            oldName = rs ? "infra2" : "left";
            break;
        case dai::CameraBoardSocket::CAM_C:
            oldName = rs ? "infra1" : "right";
            break;
        case dai::CameraBoardSocket::CAM_D:
            oldName = "cam_d";
            break;
        case dai::CameraBoardSocket::CAM_E:
            oldName = "cam_e";
            break;
        default:
            return result;
    }
    if(oldName != name) {
        result.emplace_back(body, getFrameName(prefix, oldName + "_camera_frame"));
        if(rs) result.emplace_back(body, getFrameName(prefix, oldName + "_frame"));
        result.emplace_back(optical, getOpticalFrameName(prefix, oldName, rs));
    }
    return result;
}
}  // namespace depthai_bridge::detail
