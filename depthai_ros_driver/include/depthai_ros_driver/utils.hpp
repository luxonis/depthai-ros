#pragma once

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace depthai_ros_driver {
namespace utils {
template <typename T>
T getValFromMap(const std::string& name, const std::unordered_map<std::string, T>& map) {
    try {
        return map.at(name);
    } catch(const std::out_of_range& e) {
        std::stringstream stream;
        stream << "Unable to find name " << name.c_str() << " in map.\n";
        stream << "Map values:\n";
        for(auto it = map.cbegin(); it != map.cend(); ++it) {
            stream << it->first << "\n";
        }
        throw std::out_of_range(stream.str());
    }
}
std::string getUpperCaseStr(const std::string& string);
}  // namespace utils
}  // namespace depthai_ros_driver