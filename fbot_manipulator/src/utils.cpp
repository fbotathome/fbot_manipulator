#include "fbot_manipulator/utils.hpp"
#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace fbot_manipulator {

YAML::Node Utils::readYaml(const std::string& file_path)
{
    std::filesystem::path path(file_path);
    if (path.is_relative()) {
        path = std::filesystem::current_path() / path;
    }

    if (!std::filesystem::exists(path)) {
        throw std::runtime_error("YAML file not found: " + path.string());
    }

    try {
        YAML::Node config = YAML::LoadFile(path.string());
        return config;

    } catch (const YAML::ParserException& e) {
        throw std::runtime_error("YAML Parser Error in file " + path.string() +
                                 ": " + e.what());
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to load YAML file " + path.string() +
                                 ": " + e.what());
    }
}

} // namespace fbot_manipulator
