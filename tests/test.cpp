#include <iostream>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <fstream>
#include "string"
//#include <filesystem>
#include <dirent.h>


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


int main() {

    Eigen::Matrix3d intrinsicMatrix;
    intrinsicMatrix << 608.43, 0, 322.0,
            0, 608.43, 252,
            0, 0, 1;

    // Read the JSON data from a file
    std::string filePath = "/home/ola/Desktop/transformDataList.json";
    std::ifstream file(filePath);

    // Check if the file was opened successfully
//        if (!file) {
//            std::cerr << "Failed to open the file: " << filePath << std::endl;
//            return 1;
//        }

    nlohmann::json jsonData;

    file >> jsonData;
    file.close();

    // Parse the JSON data and save it as a list of TransformData
    std::vector<TransformData> transformList;
    for (const auto &entry: jsonData) {
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

    std::vector<Eigen::MatrixXd> extrinsiclist;
    for (const auto &data: transformList) {
        // Extract the values from data
//        double q1 = data.quatX;
//        double q2 = data.quatY;
//        double q3 = data.quatZ;
//        double q0 = data.quatW;
        double q1 = 0.0;
        double q2 = 0.0;
        double q3 = 0.0;
        double q0 = 1.0;

        Eigen::MatrixXd r(3,3);
//        rotationMatrix << r11, r12, r13,
//                r21, r22, r23,
//                r31, r32, r33;

        //            a nicer way to fill r matrix
        //            d1<< current_obs.d20, current_obs.d21, current_obs.d22, current_obs.d23, current_obs.d24,
        //                    current_obs.d25, current_obs.d26, current_obs.d27, current_obs.d28 ,current_obs.d29;
        //First row of the transformation matrix
        r(0, 0) = 2 * (q0 * q0 + q1 * q1) - 1;
        r(0, 1) = 2 * (q1 * q2 - q0 * q3);
        r(0, 2) = 2 * (q1 * q3 + q0 * q2);
//        r(0, 3) = data.posX;

        // Second row of the transformation matrix
        r(1, 0) = 2 * (q1 * q2 + q0 * q3);
        r(1, 1) = 2 * (q0 * q0 + q2 * q2) - 1;
        r(1, 2) = 2 * (q2 * q3 - q0 * q1);
//        r(1, 3) = data.posY;

        // Third row of the transformation matrix
        r(2, 0) = 2 * (q1 * q3 - q0 * q2);
        r(2, 1) = 2 * (q2 * q3 + q0 * q1);
        r(2, 2) = 2 * (q0 * q0 + q3 * q3) - 1;
//        r(2, 3) = data.posZ;

//        r(3, 0) = 0;
//        r(3, 1) = 0;
//        r(3, 2) = 0;
//        r(3, 3) = 1;
        std::cout << r << std::endl;

        // Compute rotation matrix from quaternion
        Eigen::Quaterniond quat(q0, q1, q2, q3);
        Eigen::Matrix3d rotationMatrix = quat.normalized().toRotationMatrix();
        std::cout << rotationMatrix << std::endl;

        Eigen::Vector3d point3D(0.0,0, 2);
//        Eigen::Vector3d translationVector(data.posX,data.posY,data.posZ);
        Eigen::Vector3d translationVector(0,0,0);
        // Apply projection: p = K * [R | t] * P
//        Eigen::Vector3d transformedPoint = rotationMatrix * point3D + translationVector;
//        Eigen::Vector3d projectedPoint = intrinsicMatrix * transformedPoint;
        // Apply projection: p = K * [R | t] * P
        Eigen::Vector3d transformedPoint = r * point3D + translationVector;
        Eigen::Vector3d projectedPoint = intrinsicMatrix * transformedPoint;

        // Normalize projected point to obtain pixel coordinates
        double u = projectedPoint(0) / projectedPoint(2);
        double v = projectedPoint(1) / projectedPoint(2);
        std::cout << u << std::endl;
        std::cout << v << std::endl;

    }

    return 0;

}

