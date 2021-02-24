#pragma once

#include <vector>
#include "depthai/depthai.hpp"

class RgbCameraPipelineExample{

    public:
    RgbCameraPipelineExample() = default;
    ~RgbCameraPipelineExample() = default;

    void initDepthaiDev();

    std::vector<std::shared_ptr<dai::DataOutputQueue>> getExposedImageStreams();
    // std::vector<std::shared_ptr<dai::DataOutputQueue>> getExposedNnetStreams();
    
    private:
    std::vector<std::shared_ptr<dai::DataOutputQueue>> _opImageStreams;
    // std::vector<std::shared_ptr<dai::DataOutputQueue>> _opNNetStreams;

    std::unique_ptr<dai::Device> _dev;
    dai::Pipeline _p;

};
