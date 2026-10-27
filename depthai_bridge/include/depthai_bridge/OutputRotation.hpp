#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace depthai_bridge {

// Clockwise pixel rotation; also maps native optical coordinates into output coordinates.
class OutputRotation {
   public:
    explicit OutputRotation(int degrees = 0) : degrees(degrees) {
        if(degrees != 0 && degrees != 90 && degrees != 180 && degrees != 270) throw std::invalid_argument("Output rotation must be 0, 90, 180 or 270 degrees");
    }
    bool enabled() const {
        return degrees != 0;
    }
    bool swapsAxes() const {
        return degrees == 90 || degrees == 270;
    }
    std::string frame(const std::string& native) const {
        return enabled() ? native + "_rotated" : native;
    }
    template <typename T>
    std::pair<T, T> vector(T x, T y) const {
        switch(degrees) {
            case 90:
                return {-y, x};
            case 180:
                return {-x, -y};
            case 270:
                return {y, -x};
            default:
                return {x, y};
        }
    }
    template <typename T>
    std::pair<T, T> pixel(T x, T y, T width, T height) const {
        auto xy = vector(x, y);
        if(degrees == 90 || degrees == 180) xy.first += (swapsAxes() ? height : width) - 1;
        if(degrees == 180 || degrees == 270) xy.second += (swapsAxes() ? width : height) - 1;
        return xy;
    }
    std::array<float, 9> matrix(int width, int height) const {
        auto x = vector(1.f, 0.f);
        auto y = vector(0.f, 1.f);
        auto t = pixel(0.f, 0.f, float(width), float(height));
        return {x.first, y.first, t.first, x.second, y.second, t.second, 0, 0, 1};
    }
    geometry_msgs::msg::TransformStamped transform(const std::string& native) const {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = native;
        tf.child_frame_id = frame(native);
        const double halfAngle = -degrees * std::acos(-1.0) / 360.0;
        tf.transform.rotation.z = std::sin(halfAngle);
        tf.transform.rotation.w = std::cos(halfAngle);
        return tf;
    }
    // pixelsRotated: K already contains the device image homography; P's translation is still native.
    void cameraInfo(sensor_msgs::msg::CameraInfo& info, bool pixelsRotated = false) const {
        if(!enabled()) return;
        const auto q = matrix(1, 1);
        auto h = matrix(info.width, info.height);
        auto transformMatrix = [&](auto& m, int columns, bool transformed) {
            auto src = m;
            auto tmp = m;
            if(!transformed) {
                for(int i = 0; i < 3; ++i)
                    for(int j = 0; j < columns; ++j) {
                        tmp[i * columns + j] = 0;
                        for(int k = 0; k < 3; ++k) tmp[i * columns + j] += h[i * 3 + k] * src[k * columns + j];
                    }
            }
            for(int i = 0; i < 3; ++i)
                for(int j = 0; j < 3; ++j) {
                    m[i * columns + j] = 0;
                    for(int k = 0; k < 3; ++k) m[i * columns + j] += tmp[i * columns + k] * q[j * 3 + k];
                }
            if(columns == 4) {
                for(int i = 0; i < 3; ++i) {
                    m[i * columns + 3] = 0;
                    for(int k = 0; k < 3; ++k) m[i * columns + 3] += q[i * 3 + k] * src[k * columns + 3];
                }
            }
        };
        transformMatrix(info.k, 3, pixelsRotated);
        transformMatrix(info.p, 4, pixelsRotated);
        // Change both the raw and rectified optical bases.
        h = q;
        transformMatrix(info.r, 3, false);
        if(info.distortion_model == "plumb_bob" || info.distortion_model == "rational_polynomial") {
            if(info.d.size() >= 4) {
                const auto p = vector(info.d[3], info.d[2]);
                info.d[2] = p.second;
                info.d[3] = p.first;
            }
            if(info.d.size() >= 12) {
                for(int i = 0; i < 2; ++i) {
                    const auto s = vector(info.d[8 + i], info.d[10 + i]);
                    info.d[8 + i] = s.first;
                    info.d[10 + i] = s.second;
                }
            }
            if(info.d.size() >= 14 && (info.d[12] != 0 || info.d[13] != 0))
                throw std::invalid_argument("Output rotation does not support tilted-sensor distortion; use undistorted output");
        } else if(!info.d.empty() && info.distortion_model != "equidistant") {
            throw std::invalid_argument("Unsupported distortion model for output rotation: " + info.distortion_model);
        }
        if(!pixelsRotated) {
            if(info.roi.width && info.roi.height) {
                auto a = pixel(double(info.roi.x_offset), double(info.roi.y_offset), double(info.width), double(info.height));
                auto b = pixel(
                    double(info.roi.x_offset + info.roi.width - 1), double(info.roi.y_offset + info.roi.height - 1), double(info.width), double(info.height));
                info.roi.x_offset = std::min(a.first, b.first);
                info.roi.y_offset = std::min(a.second, b.second);
                if(swapsAxes()) std::swap(info.roi.width, info.roi.height);
            }
            if(swapsAxes()) std::swap(info.width, info.height);
        }
        if(swapsAxes()) std::swap(info.binning_x, info.binning_y);
        info.header.frame_id = frame(info.header.frame_id);
    }

   private:
    int degrees;
};
}  // namespace depthai_bridge
