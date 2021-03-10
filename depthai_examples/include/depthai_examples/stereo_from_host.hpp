
#pragma once

#include <iostream> // do I need this ?
#include "depthai/depthai.hpp"


class StereoHost{

 public:
    StereoHost() = default;
    ~StereoHost() = default;

    void initDepthaiDev();

    std::vector<std::shared_ptr<dai::DataOutputQueue>> getExposedOutputImageStreams();
    std::vector<std::shared_ptr<dai::DataInputQueue>> getExposedInputImageStreams();

 private:
    std::vector<std::shared_ptr<dai::DataInputQueue>> _inImageStreams;
    std::vector<std::shared_ptr<dai::DataOutputQueue>> _opImageStreams;
    std::unique_ptr<dai::Device> _dev;
    dai::Pipeline _p;

};