#pragma once

#include <sys/wait.h>

#include <array>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace depthai_bridge {
namespace detail {
// Preserve develop's replacement semantics: a custom argument string owns all
// Xacro mappings, including defaults defined by a user's custom description.
inline std::vector<std::string> descriptionCommand(const std::string& share,
                                                   const std::string& customDescription,
                                                   const std::string& customArgs,
                                                   const std::string& detectedImu,
                                                   const std::vector<std::string>& mappings,
                                                   const std::string& python = "python3") {
    std::vector<std::string> arguments = {python, share + "/scripts/resolve_description.py", "--output=json"};
    if(!customArgs.empty()) {
        arguments.push_back("--description=" + (customDescription.empty() ? share + "/urdf/base_descr.urdf.xacro" : customDescription));
        arguments.push_back("--custom-args=" + customArgs);
    } else {
        arguments.push_back("--detected-imu=" + detectedImu);
        if(!customDescription.empty()) arguments.push_back("--description=" + customDescription);
        arguments.insert(arguments.end(), mappings.begin(), mappings.end());
    }
    return arguments;
}

inline std::string shellQuote(const std::string& value) {
    std::string result = "'";
    for(char c : value) {
        result += c == '\'' ? "'\\''" : std::string(1, c);
    }
    return result + "'";
}

// The resolver emits JSON on success and diagnostics on failure. Capture both
// streams, but never accept a failed/empty result as a robot description.
inline std::string runDescriptionCommand(const std::vector<std::string>& arguments) {
    std::string command;
    for(const auto& argument : arguments) command += shellQuote(argument) + " ";
    command += "2>&1";
    std::unique_ptr<FILE, int (*)(FILE*)> pipe(popen(command.c_str(), "r"), pclose);
    if(!pipe) throw std::runtime_error("Could not start description resolver");
    std::array<char, 4096> buffer{};
    std::string output;
    while(fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) output += buffer.data();
    const bool readFailed = ferror(pipe.get());
    const int status = pclose(pipe.release());
    if(readFailed || status == -1 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        throw std::runtime_error("Description resolver failed (status " + std::to_string(status) + "): " + output);
    }
    if(output.find_first_not_of(" \t\r\n") == std::string::npos) {
        throw std::runtime_error("Description resolver returned empty output");
    }
    return output;
}
}  // namespace detail
}  // namespace depthai_bridge
