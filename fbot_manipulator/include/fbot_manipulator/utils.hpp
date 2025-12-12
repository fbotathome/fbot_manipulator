#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

namespace fbot_manipulator {

class Utils
{
public:
    // Load a YAML file and return the root node
    static YAML::Node readYaml(const std::string& file_path);
};

} // namespace fbot_manipulator
