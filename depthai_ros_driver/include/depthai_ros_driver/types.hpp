#pragma once

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace depthai_ros_driver {
namespace types {

namespace cam_types {
enum class CamName { OAK_D, OAK_D_W, OAK_D_PRO_AF, OAK_D_PRO_W, OAK_D_S2, OAK_D_LITE, OAK_1, OAK_1_W, OAK_1_LITE, OAK_1_MAX };
std::unordered_map<std::string, CamName> cam_names = {
    {"OAK-D", CamName::OAK_D},
    {"OAK-D-W", CamName::OAK_D_W},
    {"OAK-D-PRO-AF", CamName::OAK_D_PRO_AF},
    {"OAK-D-PRO-W", CamName::OAK_D_PRO_W},
    {"OAK-D-S2", CamName::OAK_D_S2},
    {"OAK-D-LITE", CamName::OAK_D_LITE},
    {"OAK-1", CamName::OAK_1},
    {"OAK-1-W", CamName::OAK_1_W},
    {"OAK-1-LITE", CamName::OAK_1_LITE},
    {"OAK-1-MAX", CamName::OAK_1_MAX},
};
class CamType {
   public:
    explicit CamType(const std::string& name) {
        auto cam_name_enum = cam_names.at(name);
        switch(cam_name_enum) {
            case CamName::OAK_D:
                stereo_ = true;
                imu_available_ = true;
                ir_available_ = false;
                break;
            case CamName::OAK_D_W:
                stereo_ = true;
                imu_available_ = true;
                ir_available_ = false;
                break;
            case CamName::OAK_D_PRO_AF:
                stereo_ = true;
                imu_available_ = true;
                ir_available_ = true;
                break;
            case CamName::OAK_D_PRO_W:
                stereo_ = true;
                imu_available_ = true;
                ir_available_ = true;
                break;
            case CamName::OAK_D_S2:
                stereo_ = true;
                imu_available_ = true;
                ir_available_ = false;
                break;
            case CamName::OAK_D_LITE:
                stereo_ = true;
                imu_available_ = true;
                ir_available_ = false;
                break;
            case CamName::OAK_1:
                stereo_ = false;
                imu_available_ = false;
                ir_available_ = false;
                break;
            case CamName::OAK_1_W:
                stereo_ = false;
                imu_available_ = false;
                ir_available_ = false;
                break;
            case CamName::OAK_1_LITE:
                stereo_ = false;
                imu_available_ = false;
                ir_available_ = false;
                break;
            case CamName::OAK_1_MAX:
                stereo_ = false;
                imu_available_ = false;
                ir_available_ = false;
                break;
            default:
                throw std::runtime_error("Unknown cam type: " + name);
        }
    }
    bool stereo() {
        return stereo_;
    }
    bool imu_available() {
        return imu_available_;
    }
    bool ir_available() {
        return ir_available_;
    }

   private:
    std::string name_;
    bool stereo_;
    bool imu_available_;
    bool ir_available_;
};
}  // namespace cam_types
}  // namespace types
}  // namespace depthai_ros_driver
