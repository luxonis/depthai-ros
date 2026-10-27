#pragma once

#include <optional>

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/Script.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "depthai_bridge/OutputRotation.hpp"

namespace depthai_ros_driver::dai_nodes::sensor_helpers {

// Rotate after native stereo/alignment and before consumers that accept rotated images.
class OutputRotation {
   public:
    OutputRotation(std::shared_ptr<dai::Pipeline> pipeline, dai::Node::Output& input, int degrees, bool opticalIntrinsics = false)
        : OutputRotation(pipeline, degrees, opticalIntrinsics) {
        input.link(this->input());
    }
    OutputRotation(std::shared_ptr<dai::Pipeline> pipeline, int degrees, bool opticalIntrinsics = false)
        : rotation(degrees), opticalIntrinsics(opticalIntrinsics), degrees(degrees), rvc2(pipeline->getDefaultDevice()->getPlatform() == dai::Platform::RVC2) {
        warp = pipeline->create<dai::node::ImageManip>();
        crop = pipeline->create<dai::node::ImageManip>();
        for(auto& manip : {warp, crop}) {
            manip->setRunOnHost(false);
            manip->inputImage.setMaxSize(1);
            manip->setBackend(dai::node::ImageManip::Backend::CPU);
        }
        control = pipeline->create<dai::node::Script>();
        control->outputs["image"].link(warp->inputImage);
        control->outputs["warp"].link(warp->inputConfig);
        control->outputs["crop"].link(crop->inputConfig);
        warp->inputConfig.setWaitForMessage(true);
        crop->inputConfig.setWaitForMessage(true);
        warp->out.link(crop->inputImage);
        if(opticalIntrinsics) {
            // ImageManip records a pixel homography; spatial nodes require diagonal K
            // in the rotated optical basis. RVC2 Script cannot edit intrinsics, so
            // exchange metadata only. Pixels stay on device; calibration changes
            // invalidate the cached conversion.
            auto correct = [rotation = rotation,
                            dx = correctionX(),
                            dy = correctionY(),
                            previous = std::optional<dai::ImgTransformation>{},
                            cached = std::shared_ptr<dai::ImgFrame>{}](std::shared_ptr<dai::ImgFrame> frame) mutable {
                const auto& transform = frame->transformation;
                if(cached && cached->getWidth() == frame->getWidth() && cached->getHeight() == frame->getHeight()
                   && previous->isEqualTransformation(transform)) {
                    return cached;
                }
                previous = transform;
                auto k = transform.getIntrinsicMatrix();
                sensor_msgs::msg::CameraInfo info;
                info.width = frame->getWidth();
                info.height = frame->getHeight();
                info.distortion_model = transform.getDistortionModel() == dai::CameraModel::Fisheye ? "equidistant" : "rational_polynomial";
                const auto distortion = transform.getDistortionCoefficients();
                // Optical intrinsics are only requested for undistorted NN/RGBD inputs.
                info.d.assign(distortion.size(), 0.0);
                for(int row = 0; row < 3; ++row)
                    for(int col = 0; col < 3; ++col) info.k[row * 3 + col] = k[row][col];
                info.k[2] -= dx;
                info.k[5] -= dy;
                rotation.cameraInfo(info, true);
                for(int row = 0; row < 3; ++row)
                    for(int col = 0; col < 3; ++col) k[row][col] = info.k[row * 3 + col];
                auto extrinsics = transform.getExtrinsics();
                const auto native = extrinsics.getRotationMatrix();
                const auto q = rotation.matrix(1, 1);
                if(native.size() == 3) extrinsics.rotationMatrix = dai::matrix::matMul(native, {{q[0], q[3], q[6]}, {q[1], q[4], q[7]}, {q[2], q[5], q[8]}});
                // Retain a common source calibration for depth and NN crops. RVC2
                // can only remap detection ROIs between images sharing that source.
                auto sourceK = transform.getSourceIntrinsicMatrix();
                sensor_msgs::msg::CameraInfo sourceInfo;
                const auto sourceSize = transform.getSourceSize();
                sourceInfo.width = sourceSize.first;
                sourceInfo.height = sourceSize.second;
                for(int row = 0; row < 3; ++row)
                    for(int col = 0; col < 3; ++col) sourceInfo.k[row * 3 + col] = sourceK[row][col];
                rotation.cameraInfo(sourceInfo);
                for(int row = 0; row < 3; ++row)
                    for(int col = 0; col < 3; ++col) sourceK[row][col] = sourceInfo.k[row * 3 + col];
                auto optical = dai::ImgTransformation(
                    sourceInfo.width, sourceInfo.height, sourceK, transform.getDistortionModel(), std::vector<float>(info.d.begin(), info.d.end()), extrinsics);
                optical.addTransformation(dai::matrix::matMul(k, dai::matrix::getMatrixInverse(sourceK)));
                optical.setSize(info.width, info.height);
                frame->transformation = optical;
                cached = frame;
                return frame;
            };
            metadata = pipeline->create<dai::node::Script>();
            crop->out.link(metadata->inputs["image"]);
            auto config = metadata->inputs["config"].createInputQueue();
            calibration = metadata->outputs["calibration"].createOutputQueue(1, false);
            calibration->addCallback([correct, config](const std::shared_ptr<dai::ADatatype>& data) mutable {
                config->send(correct(std::dynamic_pointer_cast<dai::ImgFrame>(data)));
            });
            metadata->setScript(R"(
while True:
    image = node.io["image"].get()
    header = ImgFrame(0)
    header.setWidth(image.getWidth())
    header.setHeight(image.getHeight())
    header.setTransformation(image.getTransformation())
    node.io["calibration"].send(header)
    image.setTransformation(node.io["config"].get().getTransformation())
    node.io["out"].send(image)
)");
        }
    }
    void configure(int width, int height, int encoderHeightMultiple = 0) {
        if(width <= 0 || height <= 0) throw std::invalid_argument("Output rotation requires an explicit stream width and height");
        const int ow = rotation.swapsAxes() ? height : width;
        const int oh = rotation.swapsAxes() ? width : height;
        // Parameters bound the allocation; every frame's metadata defines the actual geometry.
        const int paddedWidth = ((ow + 2 + 15) / 16) * 16;
        warp->setMaxOutputFrameSize(paddedWidth * (oh + 2) * 4);
        crop->setMaxOutputFrameSize(ow * oh * 4);
        control->setScript("encoder_height_multiple = " + std::to_string(encoderHeightMultiple) + "\nangle = " + std::to_string(degrees)
                           + "\ncx = " + std::to_string(correctionX()) + "\ncy = " + std::to_string(correctionY()) + R"(
last_size = None
while True:
    image = node.io["in"].get()
    w, h = image.getWidth(), image.getHeight()
    if last_size != (w, h):
        last_size = (w, h)
        ow, oh = (h, w) if angle != 180 else (w, h)
        if encoder_height_multiple and (ow % 32 or oh % encoder_height_multiple):
            raise ValueError("Rotated RVC2 encoder output %dx%d requires width divisible by 32 and height divisible by %d"
                             % (ow, oh, encoder_height_multiple))
        if angle == 90:
            matrix = [0, -1, h, 1, 0, 1, 0, 0, 1]
        elif angle == 180:
            matrix = [-1, 0, w, 0, -1, h, 0, 0, 1]
        else:
            matrix = [0, 1, 1, -1, 0, w, 0, 0, 1]
        # One-pixel border retains the final source row/column; compensate RVC2 sampling.
        matrix[2] += cx
        matrix[5] += cy
        warp = ImageManipConfig()
        warp.addTransformPerspective(matrix)
        warp.setOutputCenter(False)
        # Use a crop to size the canvas without recentering the transform.
        warp.addCrop(0, 0, ((ow + 17) // 16) * 16, oh + 2)
        crop = ImageManipConfig()
        crop.addCrop(1, 1, ow, oh)
    node.io["warp"].send(warp)
    node.io["crop"].send(crop)
    node.io["image"].send(image)
)");
    }
    dai::Node::Input& input() {
        return control->inputs["in"];
    }
    dai::Node::Output& out() {
        return metadata ? metadata->outputs["out"] : crop->out;
    }
    void cameraInfo(sensor_msgs::msg::CameraInfo& info, bool transformed) const {
        if(opticalIntrinsics && transformed) {
            // K and D already describe the output optical frame. Only the native
            // calibration's rectification/projection translation still needs rotation.
            const auto k = info.k;
            const auto p = info.p;
            const auto d = info.d;
            rotation.cameraInfo(info, true);
            info.k = k;
            info.d = d;
            for(int row = 0; row < 3; ++row)
                for(int col = 0; col < 3; ++col) info.p[row * 4 + col] = p[row * 4 + col];
            return;
        }
        if(transformed) {
            info.k[2] -= correctionX();
            info.p[2] -= correctionX();
            info.k[5] -= correctionY();
            info.p[6] -= correctionY();
        }
        rotation.cameraInfo(info, transformed);
    }
    depthai_bridge::OutputRotation rotation;
    const bool opticalIntrinsics;

   private:
    int correctionX() const {
        return rvc2 && (degrees == 90 || degrees == 180) ? 1 : 0;
    }
    int correctionY() const {
        return rvc2 && (degrees == 180 || degrees == 270) ? 1 : 0;
    }
    int degrees;
    bool rvc2;
    std::shared_ptr<dai::node::ImageManip> warp, crop;
    std::shared_ptr<dai::node::Script> control, metadata;
    std::shared_ptr<dai::MessageQueue> calibration;
};
}  // namespace depthai_ros_driver::dai_nodes::sensor_helpers
