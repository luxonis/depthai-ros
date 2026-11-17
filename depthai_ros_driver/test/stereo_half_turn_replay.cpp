// Diagnostic only: replay a frozen rectified pair through RVC2 stereo hardware.
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <vector>

#include "depthai/depthai.hpp"

namespace {
std::shared_ptr<dai::ImgFrame> frame(const cv::Mat& image,
                                     dai::CameraBoardSocket socket,
                                     const std::array<std::array<float, 3>, 3>& k,
                                     int sequence,
                                     std::chrono::steady_clock::time_point timestamp) {
    auto result = std::make_shared<dai::ImgFrame>();
    result->setType(dai::ImgFrame::Type::GRAY8);
    result->setWidth(image.cols);
    result->setHeight(image.rows);
    result->setStride(image.cols);
    result->setInstanceNum(unsigned(socket));
    result->setTransformation(dai::ImgTransformation(image.cols, image.rows, k));
    result->setData(std::vector<uint8_t>(image.data, image.data + image.total()));
    result->setSequenceNum(sequence);
    result->setTimestamp(timestamp);
    return result;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        if(argc != 6)
            throw std::invalid_argument("Usage: stereo_half_turn_replay DEVICE FROZEN_PAIR_DIR OUTPUT_DIR PROFILE native|vertical|horizontal|half-turn");
        const std::filesystem::path source = argv[2], out = argv[3];
        const std::string profile = argv[4], orientation = argv[5];
        const std::vector<std::string> profiles = {"stock", "stock-no-v", "raw", "raw-no-v", "raw-no-h"};
        if(std::find(profiles.begin(), profiles.end(), profile) == profiles.end()) throw std::invalid_argument("Unknown profile");
        if(orientation != "native" && orientation != "vertical" && orientation != "horizontal" && orientation != "half-turn")
            throw std::invalid_argument("Unknown orientation");
        std::filesystem::create_directories(out);
        cv::Mat left = cv::imread((source / "left.png").string(), cv::IMREAD_GRAYSCALE);
        cv::Mat right = cv::imread((source / "right.png").string(), cv::IMREAD_GRAYSCALE);
        if(left.empty() || right.empty() || left.size() != right.size()) throw std::invalid_argument("Need equal-size frozen left.png/right.png");
        const int width = left.cols, height = left.rows;
        nlohmann::json reference;
        std::ifstream(source / "configuration.json") >> reference;
        auto flatK = reference.at("custom_rect_K").get<std::vector<float>>();
        const float baseline = reference.at("baseline_cm");
        const bool swap = orientation == "horizontal" || orientation == "half-turn";
        int flipCode = orientation == "vertical" ? 0 : orientation == "horizontal" ? 1 : -1;
        if(orientation != "native") {
            cv::flip(left, left, flipCode);
            cv::flip(right, right, flipCode);
        }
        if(swap) std::swap(left, right);
        // Virtual calibration describes the already rectified images; it is
        // pipeline-local and never written to EEPROM. No rectification is run.
        if(orientation == "vertical" || orientation == "half-turn") flatK[5] = height - 1 - flatK[5];
        if(swap) flatK[2] = width - 1 - flatK[2];
        std::array<std::array<float, 3>, 3> k;
        std::vector<std::vector<float>> kv(3, std::vector<float>(3));
        for(int r = 0; r < 3; ++r)
            for(int c = 0; c < 3; ++c) k[r][c] = kv[r][c] = flatK[r * 3 + c];
        const std::vector<std::vector<float>> identity = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        dai::CalibrationHandler calibration;
        for(auto socket : {dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C}) {
            calibration.setCameraIntrinsics(socket, kv, width, height);
            calibration.setDistortionCoefficients(socket, std::vector<float>(14, 0));
            calibration.setCameraType(socket, dai::CameraModel::Perspective);
        }
        calibration.setCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, identity, {-baseline, 0, 0}, {-baseline, 0, 0});
        calibration.setStereoLeft(dai::CameraBoardSocket::CAM_B, identity);
        calibration.setStereoRight(dai::CameraBoardSocket::CAM_C, identity);
        auto device = std::make_shared<dai::Device>(dai::DeviceInfo(argv[1]));
        if(device->getPlatform() != dai::Platform::RVC2) throw std::invalid_argument("RVC2-only diagnosis");
        auto pipeline = std::make_shared<dai::Pipeline>(device);
        pipeline->setCalibrationData(calibration);
        auto stereo = pipeline->create<dai::node::StereoDepth>();
        stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::FAST_ACCURACY);
        stereo->setInputResolution(width, height);
        stereo->setRectification(false);
        stereo->setFocalLength(flatK[0]);
        stereo->setBaseline(baseline);
        stereo->setDisparityToDepthUseSpecTranslation(false);
        const bool lr = profile.rfind("raw", 0) != 0;
        const bool subpixel = profile.rfind("raw", 0) != 0;
        stereo->setLeftRightCheck(lr);
        stereo->setSubpixel(subpixel);
        stereo->initialConfig->setSubpixelFractionalBits(3);
        stereo->initialConfig->setLeftRightCheckThreshold(10);
        stereo->initialConfig->setConfidenceThreshold(15);
        stereo->setDepthAlign(swap ? dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_RIGHT
                                   : dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_LEFT);
        if(profile != "stock" && profile != "stock-no-v") {
            auto& post = stereo->initialConfig->postProcessing;
            post.median = dai::StereoDepthConfig::MedianFilter::MEDIAN_OFF;
            post.spatialFilter.enable = false;
            post.temporalFilter.enable = false;
            post.speckleFilter.enable = false;
            post.holeFilling.enable = false;
            post.adaptiveMedianFilter.enable = false;
            post.decimationFilter.decimationFactor = 1;
        }
        auto& aggregation = stereo->initialConfig->costAggregation;
        if(profile == "raw-no-v" || profile == "stock-no-v") {
            aggregation.verticalPenaltyCostP1 = 0;
            aggregation.verticalPenaltyCostP2 = 0;
        }
        if(profile == "raw-no-h") {
            aggregation.horizontalPenaltyCostP1 = 0;
            aggregation.horizontalPenaltyCostP2 = 0;
        }
        auto iqLeft = stereo->left.createInputQueue(), iqRight = stereo->right.createInputQueue();
        std::map<std::string, std::shared_ptr<dai::MessageQueue>> queues;
        queues["depth"] = stereo->depth.createOutputQueue(4, false);
        queues["disparity"] = stereo->disparity.createOutputQueue(4, false);
        queues["confidence"] = stereo->confidenceMap.createOutputQueue(4, false);
        if(lr) {
            queues["lr1"] = stereo->debugDispLrCheckIt1.createOutputQueue(4, false);
            queues["lr2"] = stereo->debugDispLrCheckIt2.createOutputQueue(4, false);
        }
        nlohmann::json result = {{"profile", profile},
                                 {"orientation", orientation},
                                 {"width", width},
                                 {"height", height},
                                 {"lr", lr},
                                 {"subpixel", subpixel},
                                 {"reference_dir", source.string()},
                                 {"rectification", false},
                                 {"focal", flatK[0]},
                                 {"baseline_cm", baseline},
                                 {"os_version", device->getOSVersion()},
                                 {"config", *stereo->initialConfig}};
        cv::imwrite((out / "input-left.png").string(), left);
        cv::imwrite((out / "input-right.png").string(), right);
        pipeline->start();
        for(int sequence = 1; sequence <= 5; ++sequence) {
            const auto timestamp = std::chrono::steady_clock::now();
            iqLeft->send(frame(left, dai::CameraBoardSocket::CAM_B, k, sequence, timestamp));
            iqRight->send(frame(right, dai::CameraBoardSocket::CAM_C, k, sequence, timestamp));
            for(auto& [name, queue] : queues) {
                bool timedOut = false;
                auto output = queue->get<dai::ImgFrame>(std::chrono::seconds(10), timedOut);
                if(timedOut || !output) throw std::runtime_error("Missing " + name);
                if(output->getSequenceNum() != sequence) throw std::runtime_error("Sequence mismatch in " + name);
                if(sequence >= 3) cv::imwrite((out / (name + "-" + std::to_string(sequence) + ".png")).string(), output->getFrame());
                result["outputs"][name] = {{"type", int(output->getType())}, {"width", output->getWidth()}, {"height", output->getHeight()}};
            }
        }
        pipeline->stop();
        pipeline->wait();
        std::ofstream(out / "result.json") << result.dump(2);
        std::cout << "PASS " << profile << ' ' << orientation << std::endl;
        return 0;
    } catch(const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
