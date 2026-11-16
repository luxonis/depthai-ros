#pragma once

#include <unordered_map>

#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/output_rotation.hpp"

namespace depthai_ros_driver::dai_nodes::nn_helpers {

// Request the inverse aspect ratio: camera cropping/resizing followed by a quarter turn
// is equivalent to rotating first and then cropping/resizing to the model's input size.
class RotatedInput {
   public:
    RotatedInput(std::shared_ptr<dai::Pipeline> pipeline,
                 std::shared_ptr<dai::node::Camera> camera,
                 dai::node::NeuralNetwork& network,
                 int degrees,
                 bool spatial = false)
        : degrees(degrees) {
        const auto nnArchive = network.getNNArchive();
        const auto& archive = nnArchive.value().get();
        auto size = archive.getInputSize().value();
        depthai_bridge::OutputRotation rotation(degrees);
        if(rotation.swapsAxes()) std::swap(size.first, size.second);
        auto type = pipeline->getDefaultDevice()->getPlatform() == dai::Platform::RVC2 ? dai::ImgFrame::Type::BGR888p : dai::ImgFrame::Type::BGR888i;
        const auto& inputs = archive.getConfig<dai::nn_archive::v1::Config>().model.inputs;
        if(inputs.size() != 1) throw std::invalid_argument("Rotated NN input requires a single image input");
        if(inputs.front().preprocessing.daiType) {
            static const std::unordered_map<std::string, dai::ImgFrame::Type> types = {
                {"BGR888p", dai::ImgFrame::Type::BGR888p},
                {"RGB888p", dai::ImgFrame::Type::RGB888p},
                {"BGR888i", dai::ImgFrame::Type::BGR888i},
                {"RGB888i", dai::ImgFrame::Type::RGB888i},
                {"GRAY8", dai::ImgFrame::Type::GRAY8},
                {"NV12", dai::ImgFrame::Type::NV12},
                {"YUV420p", dai::ImgFrame::Type::YUV420p},
            };
            const auto requested = types.find(*inputs.front().preprocessing.daiType);
            if(requested == types.end()) throw std::invalid_argument("Unsupported rotated NN input type: " + *inputs.front().preprocessing.daiType);
            type = requested->second;
        }
        native = camera->requestOutput(size, type, dai::ImgResizeMode::CROP, std::nullopt, true);
        image = std::make_shared<sensor_helpers::OutputRotation>(pipeline, *native, degrees, spatial);
        image->configure(size.first, size.second);
        image->out().link(network.input);
    }

    void linkDepth(std::shared_ptr<dai::Pipeline> pipeline,
                   dai::node::SpatialDetectionNetwork& network,
                   dai::node::StereoDepth& stereo,
                   bool alreadyAligned,
                   int width,
                   int height,
                   bool alignOnHost) {
        // Align in native camera coordinates, then rotate both spatial NN inputs.
        dai::Node::Output* aligned = &stereo.depth;
        if(!alreadyAligned) {
            auto* align = network.depthAlign ? &**network.depthAlign : pipeline->create<dai::node::ImageAlign>().get();
            align->setRunOnHost(alignOnHost);
            stereo.depth.link(align->input);
            native->link(align->inputAlignTo);
            aligned = &align->outputAligned;
        }
        const auto nnArchive = network.neuralNetwork->getNNArchive();
        auto size = nnArchive->get().getInputSize().value();
        if(image->rotation.swapsAxes()) std::swap(size.first, size.second);
        // Stereo may already be aligned to a larger RGB output. Reserve for both.
        depth = std::make_shared<sensor_helpers::OutputRotation>(pipeline, *aligned, degrees, true);
        depth->configure(std::max<int>(size.first, width), std::max<int>(size.second, height));
        depth->out().link(network.inputDepth);
    }

    std::shared_ptr<sensor_helpers::OutputRotation> image, depth;

   private:
    int degrees;
    dai::Node::Output* native;
};
}  // namespace depthai_ros_driver::dai_nodes::nn_helpers
