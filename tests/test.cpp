#include <iostream>
//#include "json.hpp"
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <fstream>


struct TransformData {
    double posX;
    double posY;
    double posZ;
    double quatX;
    double quatY;
    double quatZ;
    double quatW;
    std::string name;
};

int main(){

    // Read the JSON data from a file
    std::string filePath = "/home/ola/Desktop/transformDataList.json";
    std::ifstream file(filePath);

    // Check if the file was opened successfully
    if (!file)
    {
        std::cerr << "Failed to open the file: " << filePath << std::endl;
        return 1;
    }

    nlohmann::json jsonData;

    file >> jsonData;

    // Parse the JSON data and save it as a list of TransformData
    std::vector<TransformData> transformList;
    for (const auto& entry : jsonData)
    {
        TransformData transform;
        transform.posX = entry["posX"];
        transform.posY = entry["posY"];
        transform.posZ = entry["posZ"];
        transform.quatX = entry["quatX"];
        transform.quatY = entry["quatY"];
        transform.quatZ = entry["quatZ"];
        transform.quatW = entry["quatW"];
        transform.name = entry["name"];
        transformList.push_back(transform);
    }

    for (const auto& data : transformList)
    {
    // Extract the values from data
    double q1 = data.quatX;
    double q2 = data.quatY;
    double q3 = data.quatZ;
    double q0 = data.quatW;

    Eigen::MatrixXd r(4,4);
    //First row of the transformation matrix
    r(0,0) = 2 * (q0 * q0 + q1 * q1) - 1;
    r(0,1)  = 2 * (q1 * q2 - q0 * q3);
    r(0,2) = 2 * (q1 * q3 + q0 * q2);
    r(0,3) = data.posX;

    // Second row of the transformation matrix
    r(1,0) = 2 * (q1 * q2 + q0 * q3);
    r(1,1) = 2 * (q0 * q0 + q2 * q2) - 1;
    r(1,2) = 2 * (q2 * q3 - q0 * q1);
    r(1,3) = data.posY;

    // Third row of the transformation matrix
    r(2,0) = 2 * (q1 * q3 - q0 * q2);
    r(2,1) = 2 * (q2 * q3 + q0 * q1);
    r(2,2) = 2 * (q0 * q0 + q3 * q3) - 1;
    r(2,3) = data.posZ;

    r(3,0) = 0;
    r(3,1) = 0;
    r(3,2) = 0;
    r(3,3) = 1;
    std::cout << r << std::endl;
    }
    return 0;
}