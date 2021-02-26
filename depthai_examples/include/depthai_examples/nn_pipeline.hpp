#pragma once

#include <vector>
#include "depthai/depthai.hpp"

class MobileNetDetectionExample{

    public:

    static const std::vector<std::string> label_map ;

    MobileNetDetectionExample() = default;
    ~MobileNetDetectionExample() = default;

    void initDepthaiDev(std::string nnPath);

    std::vector<std::shared_ptr<dai::DataOutputQueue>> getExposedImageStreams();
    std::vector<std::shared_ptr<dai::DataOutputQueue>> getExposedNnetStreams();
    
    private:
    std::vector<std::shared_ptr<dai::DataOutputQueue>> _opImageStreams;
    std::vector<std::shared_ptr<dai::DataOutputQueue>> _opNNetStreams;

    std::unique_ptr<dai::Device> _dev;
    dai::Pipeline _p;

};