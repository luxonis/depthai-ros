#pragma once

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
#include "depthai/pipeline/node/Script.hpp"
#include "depthai_bridge/OutputRotation.hpp"

namespace depthai_ros_driver::dai_nodes::sensor_helpers {

// Keep this branch separate from native stereo, alignment, RGBD and VIO inputs.
class OutputRotation {
   public:
    OutputRotation(std::shared_ptr<dai::Pipeline> pipeline, dai::Node::Output& input, int degrees)
        : rotation(degrees), degrees(degrees), rvc2(pipeline->getDefaultDevice()->getPlatform() == dai::Platform::RVC2) {
        warp = pipeline->create<dai::node::ImageManip>();
        crop = pipeline->create<dai::node::ImageManip>();
        for(auto& manip : {warp, crop}) {
            manip->setRunOnHost(false);
            manip->setBackend(dai::node::ImageManip::Backend::CPU);
        }
        control = pipeline->create<dai::node::Script>();
        input.link(control->inputs["in"]);
        control->outputs["image"].link(warp->inputImage);
        control->outputs["warp"].link(warp->inputConfig);
        control->outputs["crop"].link(crop->inputConfig);
        warp->inputConfig.setWaitForMessage(true);
        crop->inputConfig.setWaitForMessage(true);
        warp->out.link(crop->inputImage);
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
while True:
    image = node.io["in"].get()
    w, h = image.getWidth(), image.getHeight()
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
    dai::Node::Output& out() {
        return crop->out;
    }
    void cameraInfo(sensor_msgs::msg::CameraInfo& info, bool transformed) const {
        if(transformed) {
            info.k[2] -= correctionX();
            info.p[2] -= correctionX();
            info.k[5] -= correctionY();
            info.p[6] -= correctionY();
        }
        rotation.cameraInfo(info, transformed);
    }
    depthai_bridge::OutputRotation rotation;

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
    std::shared_ptr<dai::node::Script> control;
};
}  // namespace depthai_ros_driver::dai_nodes::sensor_helpers
