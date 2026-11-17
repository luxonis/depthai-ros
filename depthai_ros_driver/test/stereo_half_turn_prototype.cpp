// Isolated RVC2 experiment. No production pipeline behavior is changed.
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <thread>

#include "depthai/depthai.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/output_rotation.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_rotation.hpp"

namespace {
constexpr int width = 640, height = 400, step = 16;
cv::Mat matrix(const std::vector<std::vector<float>>& source) {
    cv::Mat result(int(source.size()), int(source.front().size()), CV_64F);
    for(int r = 0; r < result.rows; ++r)
        for(int c = 0; c < result.cols; ++c) result.at<double>(r, c) = source[r][c];
    return result;
}
std::vector<double> values(const cv::Mat& source) {
    cv::Mat contiguous = source.clone();
    return {contiguous.ptr<double>(), contiguous.ptr<double>() + contiguous.total()};
}
std::vector<uint8_t> mesh(const cv::Mat& k, const std::vector<float>& d, const cv::Mat& r, const cv::Mat& p, bool outputFlip, bool sensorFlip) {
    std::vector<float> points;
    for(int y = 0; y <= height; y += step) {
        for(int x = 0; x <= width; x += step) {
            // RVC2 rectifier samples at pixel centers. Reversing the mesh needs
            // width-x/height-y, not width-1-x/height-1-y, to match a half turn
            // of its native raster (verified by captured feature coordinates).
            const int ox = outputFlip ? width - x : x;
            const int oy = outputFlip ? height - y : y;
            cv::Mat pixel = (cv::Mat_<double>(3, 1) << ox, oy, 1);
            cv::Mat ray = r.t() * p.inv() * pixel;
            std::vector<cv::Point2d> distorted;
            cv::projectPoints(std::vector<cv::Point3d>{{ray.at<double>(0), ray.at<double>(1), ray.at<double>(2)}}, cv::Vec3d(), cv::Vec3d(), k, d, distorted);
            float sx = float(distorted[0].x), sy = float(distorted[0].y);
            if(sensorFlip) {
                sx = width - 1 - sx;
                sy = height - 1 - sy;
            }
            points.push_back(sy);
            points.push_back(sx);
        }
    }
    std::vector<uint8_t> bytes(points.size() * sizeof(float));
    std::memcpy(bytes.data(), points.data(), bytes.size());
    return bytes;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        if(argc != 5)
            throw std::invalid_argument("Usage: stereo_half_turn_prototype DEVICE native|current|mesh-native|sensor-flip|fused-flip SECONDS OUTPUT_DIR");
        const std::string mode = argv[2];
        if(mode != "native" && mode != "current" && mode != "mesh-native" && mode != "sensor-flip" && mode != "fused-flip")
            throw std::invalid_argument("Unknown mode");
        const double seconds = std::stod(argv[3]);
        if(seconds < 5) throw std::invalid_argument("Measure at least five seconds after warmup");
        const std::filesystem::path out = argv[4];
        std::filesystem::create_directories(out);
        auto device = std::make_shared<dai::Device>(dai::DeviceInfo(argv[1]));
        if(device->getPlatform() != dai::Platform::RVC2) throw std::invalid_argument("Prototype is RVC2-only");
        auto pipeline = std::make_shared<dai::Pipeline>(device);
        const auto cal = device->readCalibration();
        const auto kl = matrix(cal.getCameraIntrinsics(dai::CameraBoardSocket::CAM_B, width, height));
        const auto kr = matrix(cal.getCameraIntrinsics(dai::CameraBoardSocket::CAM_C, width, height));
        const auto dl = cal.getDistortionCoefficients(dai::CameraBoardSocket::CAM_B);
        const auto dr = cal.getDistortionCoefficients(dai::CameraBoardSocket::CAM_C);
        const auto extrinsics = matrix(cal.getCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, false));
        cv::Mat r1, r2, p1, p2, q;
        const cv::Mat rotation = extrinsics(cv::Rect(0, 0, 3, 3)).clone();
        const cv::Mat translation = extrinsics(cv::Rect(3, 0, 1, 3)).clone();
        cv::stereoRectify(kl, dl, kr, dr, {width, height}, rotation, translation, r1, r2, p1, p2, q, cv::CALIB_ZERO_DISPARITY, 0);
        cv::Mat rectK = p1(cv::Rect(0, 0, 3, 3)).clone();
        const bool flipped = mode == "sensor-flip" || mode == "fused-flip";
        const bool custom = flipped || mode == "mesh-native";
        const bool sensorFlip = mode == "sensor-flip";
        auto left = pipeline->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
        auto right = pipeline->create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
        for(auto camera : {left, right}) {
            camera->initialControl.setManualExposure(10000, 400);
            camera->setIspNumFramesPool(8);
            camera->setOutputsNumFramesPool(8);
            if(sensorFlip) {
                auto native = dai::CameraImageOrientation::AUTO;
                for(const auto& feature : device->getConnectedCameraFeatures()) {
                    if(feature.socket == camera->getBoardSocket()) native = feature.orientation;
                }
                // This explicit baseline was verified against AUTO with real
                // images on 1944301071834D1300 in test_sensor_orientation.py.
                if(native == dai::CameraImageOrientation::AUTO) {
                    if(std::string(argv[1]) != "1944301071834D1300") throw std::invalid_argument("Unknown native sensor orientation on this device");
                    native = dai::CameraImageOrientation::NORMAL;
                }
                camera->setImageOrientation(*depthai_ros_driver::dai_nodes::sensor_helpers::halfTurnOrientation(native));
            }
        }
        auto* leftOut = left->requestOutput({width, height}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, 30.f, false);
        auto* rightOut = right->requestOutput({width, height}, dai::ImgFrame::Type::GRAY8, dai::ImgResizeMode::CROP, 30.f, false);
        auto stereo = pipeline->create<dai::node::StereoDepth>();
        stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::FAST_ACCURACY);
        stereo->setLeftRightCheck(true);
        stereo->setSubpixel(true);
        stereo->initialConfig->setSubpixelFractionalBits(3);
        stereo->initialConfig->setLeftRightCheckThreshold(10);
        stereo->initialConfig->setConfidenceThreshold(15);
        stereo->setNumFramesPool(8);
        stereo->enableDistortionCorrection(true);
        stereo->setInputResolution(width, height);
        // Always reference physical CAM_B: it moves to the logical right port
        // after swapping inputs to retain positive horizontal disparity.
        stereo->setDepthAlign(flipped ? dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_RIGHT
                                      : dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_LEFT);
        (flipped ? rightOut : leftOut)->link(stereo->left);
        (flipped ? leftOut : rightOut)->link(stereo->right);
        if(custom) {
            auto ml = mesh(kl, dl, r1, rectK, flipped, sensorFlip);
            auto mr = mesh(kr, dr, r2, rectK, flipped, sensorFlip);
            stereo->loadMeshData(flipped ? mr : ml, flipped ? ml : mr);
            stereo->setMeshStep(step, step);
            stereo->setFocalLength(float(rectK.at<double>(0, 0)));
            stereo->setBaseline(float(cv::norm(translation)));
            stereo->setDisparityToDepthUseSpecTranslation(false);
        }
        std::map<std::string, dai::Node::Output*> outputs = {{"depth", &stereo->depth},
                                                             {"left", flipped ? &stereo->rectifiedRight : &stereo->rectifiedLeft},
                                                             {"right", flipped ? &stereo->rectifiedLeft : &stereo->rectifiedRight}};
        const auto* streamSelection = std::getenv("DEPTHAI_STEREO_PROTOTYPE_STREAMS");
        const bool depthOnly = streamSelection && std::string(streamSelection) == "depth";
        if(depthOnly) {
            outputs.erase("left");
            outputs.erase("right");
        }
        std::vector<std::shared_ptr<depthai_ros_driver::dai_nodes::sensor_helpers::OutputRotation>> warps;
        if(mode == "current") {
            for(auto& [name, output] : outputs) {
                auto warp = std::make_shared<depthai_ros_driver::dai_nodes::sensor_helpers::OutputRotation>(pipeline, *output, 180);
                warp->input().setBlocking(false);
                warp->input().setMaxSize(1);
                warp->configure(width, height);
                output = &warp->out();
                warps.push_back(warp);
            }
        }
        std::map<std::string, std::shared_ptr<dai::MessageQueue>> queues;
        for(auto& [name, output] : outputs) queues[name] = output->createOutputQueue(4, false);
        nlohmann::json result = {{"mode", mode},
                                 {"streams_requested", depthOnly ? "depth" : "all"},
                                 {"production_ready", false},
                                 {"source_isp_pool", 8},
                                 {"device", argv[1]},
                                 {"width", width},
                                 {"height", height},
                                 {"duration_requested", seconds},
                                 {"lr_check", true},
                                 {"subpixel", true},
                                 {"physical_depth_reference", "CAM_B"},
                                 {"custom_mesh", custom},
                                 {"sensor_readout_flip", sensorFlip},
                                 {"baseline_cm", cv::norm(translation)},
                                 {"custom_rect_K", values(rectK)},
                                 {"rect_R_B", values(r1)},
                                 {"rect_R_C", values(r2)}};
        std::ofstream(out / "configuration.json") << result.dump(2);
        pipeline->start();
        std::map<std::string, size_t> counts, gaps;
        std::map<std::string, int64_t> previous;
        std::map<std::string, cv::Mat> latest;
        const auto warmupEnd = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        auto measureStart = warmupEnd;
        auto nextSnapshot = warmupEnd + std::chrono::seconds(1);
        const auto end = warmupEnd + std::chrono::milliseconds(int(seconds * 1000));
        int snapshot = 0;
        while(std::chrono::steady_clock::now() < end) {
            bool received = false;
            for(auto& [name, queue] : queues) {
                for(const auto& frame : queue->tryGetAll<dai::ImgFrame>()) {
                    received = true;
                    latest[name] = frame->getFrame().clone();
                    if(std::chrono::steady_clock::now() < measureStart) continue;
                    ++counts[name];
                    const auto seq = frame->getSequenceNum();
                    if(previous.count(name) && seq > previous[name] + 1) gaps[name] += seq - previous[name] - 1;
                    previous[name] = seq;
                }
            }
            if(std::chrono::steady_clock::now() >= nextSnapshot && latest.size() == outputs.size()) {
                for(const auto& [name, image] : latest) cv::imwrite((out / (name + "-" + std::to_string(snapshot) + ".png")).string(), image);
                ++snapshot;
                nextSnapshot += std::chrono::seconds(1);
            }
            if(!received) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - measureStart).count();
        pipeline->stop();
        pipeline->wait();
        result["duration_measured"] = elapsed;
        result["snapshots"] = snapshot;
        for(const auto& [name, output] : outputs) {
            result["streams"][name] = {{"frames", counts[name]}, {"fps", counts[name] / elapsed}, {"sequence_gaps", gaps[name]}};
            if(!latest.count(name) || counts[name] < 5) throw std::runtime_error("Missing output " + name);
            if(name == "depth") {
                result["depth_valid_fraction"] = double(cv::countNonZero(latest[name])) / latest[name].total();
                if(cv::countNonZero(latest[name]) < 100) throw std::runtime_error("No usable depth");
            }
        }
        std::ofstream(out / "result.json") << result.dump(2);
        std::cout << result.dump(2) << std::endl;
        return 0;
    } catch(const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
