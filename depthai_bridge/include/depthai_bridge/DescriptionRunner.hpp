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
