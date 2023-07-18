//
// Created by ola on 7/17/23.
//
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <map>

int main()
{
    // Load YAML file
    YAML::Node yaml_data = YAML::LoadFile("/home/ola/smart_home/src/smart-home/external/particle_filter/src/yaml.yaml");

    // Initialize map
    std::map<std::string, Eigen::Matrix4d> transformations_map;

    // Extract transformations from YAML data
    YAML::Node transformations = yaml_data["transformations"];
    for (const auto& transformation : transformations)
    {
        // Extract ID and matrix
        std::string matrix_id = transformation["id"].as<std::string>();
        Eigen::Matrix4d matrix;
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                matrix(i, j) = transformation["matrix"][i][j].as<double>();
            }
        }

        // Store in map
        transformations_map[matrix_id] = matrix;
    }
    std::cout << "Transformations mapfffff:\n";
        std::cout << transformations_map["matrix1"] << std::endl;

    // Print the map
    std::cout << "Transformations map:\n";
    for (const auto& pair : transformations_map)
    {
        std::cout << "ID: " << pair.first << "\n" << pair.second << std::endl;
    }

    return 0;
}
