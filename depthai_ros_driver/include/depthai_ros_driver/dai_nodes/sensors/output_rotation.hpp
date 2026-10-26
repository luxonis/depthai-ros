#pragma once

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"
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
        input.link(warp->inputImage);
        warp->out.link(crop->inputImage);
    }
    void configure(int width, int height) {
        if(width <= 0 || height <= 0) throw std::invalid_argument("Output rotation requires an explicit stream width and height");
        const int ow = rotation.swapsAxes() ? height : width;
        const int oh = rotation.swapsAxes() ? width : height;
        auto matrix = rotation.matrix(width, height);
        // Warp crop bounds use pixel edges. A one-pixel border retains the last source row/column.
        // RVC2 samples reflected axes one pixel earlier than its reported transformation.
        matrix[2] += 1 + correctionX();
        matrix[5] += 1 + correctionY();
        warp->initialConfig->addTransformPerspective(matrix);
        const int paddedWidth = ((ow + 2 + 15) / 16) * 16;
        warp->initialConfig->setOutputSize(paddedWidth, oh + 2);
        warp->initialConfig->base.center = false;
        warp->setMaxOutputFrameSize(paddedWidth * (oh + 2) * 4);
        crop->initialConfig->addCrop(1, 1, ow, oh);
        crop->setMaxOutputFrameSize(ow * oh * 4);
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
};
}  // namespace depthai_ros_driver::dai_nodes::sensor_helpers
