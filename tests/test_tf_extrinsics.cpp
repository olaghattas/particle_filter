////
//// Created by ola on 7/10/23.
////

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <nlohmann/json.hpp>

#include <random>
#include <map>
#include <array>

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

struct Pose {
    geometry_msgs::msg::Vector3 translation;
    geometry_msgs::msg::Quaternion quaternion;
};


class ParticleFilterNode : public rclcpp::Node {
private:

    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cameraMatrix;
    Eigen::Matrix<double, 3, 4, Eigen::RowMajor> cameraProjectionMatrix;
    std::map<std::string, Pose> cameraextrinsics;



public:
    ParticleFilterNode() : Node("check_extrinsics") {

        auto cameraInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera_dining_room/color/camera_info", 1,
                [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                    cameraMatrix = cameraInfoCallback(msg);
                });
        bool sim = true;
        // to decide, from unity i have to subscribe to tf2, thinking of keepng it the same or getting extrininsc directly form checkerboard.

        if (sim) {
            auto tf_subscriber_ = create_subscription<geometry_msgs::msg::TransformStamped>(
                    "/tf", 1, [this](const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
                        cameraextrinsics = tfCallback(msg);
                    });
        }

    }


        Eigen::Matrix<double, 3, 3, Eigen::RowMajor>
        cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr &msg) {
            // Access camera matrix values
            Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> K(msg->k.data());
            cameraMatrix = K;

            // Access camera projection matrix values might be used later
//        Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> P(msg->p.data());
//        cameraProjectionMatrix = P;
            return cameraMatrix;
        }

        Pose get_extrinisics(const sensor_msgs::msg::Image::SharedPtr msg) {
            Pose cam_ext;
            geometry_msgs::msg::Vector3 translation;
            translation.x = 1.0;
            translation.y = 2.0;
            translation.z = 3.0;
            cam_ext.translation = translation;
            geometry_msgs::msg::Quaternion rotation;
            rotation.x = 0.0;
            rotation.y = 0.0;
            rotation.z = 0.0;
            rotation.w = 1.0;
            cam_ext.quaternion = rotation;
            return cam_ext;
        }

        Pose get_pose_from_transform(const geometry_msgs::msg::TransformStamped::SharedPtr &msg) {
            Pose cam_ext;
            geometry_msgs::msg::Vector3 translation;
            cam_ext.translation = msg->transform.translation;
            geometry_msgs::msg::Quaternion rotation;
            cam_ext.quaternion = msg->transform.rotation;
            return cam_ext;
        }

        std::map<std::string, Pose> tfCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
            std::map<std::string, Pose> cameraextrinsics;
            // Check the child_frame_id
            std::string child_frame_id = msg->child_frame_id;
            if (child_frame_id == "tapo_camera_kitchen") {
                // Perform your desired actions here
                Pose result_kitchen = get_pose_from_transform(msg);
                std::string kitchen = "kitchen";
                cameraextrinsics.insert(std::make_pair(kitchen, result_kitchen));

            }
            if (child_frame_id == "tapo_camera_dining") {
                // Perform your desired actions here
                Pose result_dining = get_pose_from_transform(msg);
                std::string dine = "dining";
                cameraextrinsics.insert(std::make_pair(dine, result_dining));

            }
            if (child_frame_id == "tapo_camera_bedroom") {
                // Perform your desired actions here
                Pose result_bedroom = get_pose_from_transform(msg);
                std::string bedroom = "bedroom";
                cameraextrinsics.insert(std::make_pair(bedroom, result_bedroom));

            }
            if (child_frame_id == "tapo_camera_living") {
                // Perform your desired actions here
                Pose result_living = get_pose_from_transform(msg);
                std::string living = "living";
                cameraextrinsics.insert(std::make_pair(living, result_living));

            }
            return cameraextrinsics;
        }

        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> getCameraMatrix() const {
            return cameraMatrix;
        }

        std::map<std::string, Pose> getCameracExtrinsics() const {
            return cameraextrinsics;
        }

        void printCameraMatrices(Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cameraMatrix) {
            std::cout << "Camera Intrinsic Matrix:\n" << cameraMatrix << std::endl;

            if (!cameraextrinsics.empty()) {
                std::cout << "Camera Extrinsic Matrices:" << std::endl;
                for (const auto& extrinsic : cameraextrinsics) {
                    std::cout << "Child Frame ID: " << extrinsic.first << std::endl;
                    std::cout << "Translation: ["
                              << extrinsic.second.translation.x << ", "
                              << extrinsic.second.translation.y << ", "
                              << extrinsic.second.translation.z << "]" << std::endl;
                    std::cout << "Quaternion: ["
                              << extrinsic.second.quaternion.x << ", "
                              << extrinsic.second.quaternion.y << ", "
                              << extrinsic.second.quaternion.z << ", "
                              << extrinsic.second.quaternion.w << "]" << std::endl;
                    std::cout << "-----------------------" << std::endl;
                }
            }
        }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParticleFilterNode>();
    std::map<std::string, Pose> cameraextrinsics_;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cameraMatrix_;

    // subscribe to camera info to get intrinsic parameters and save them in a map
    cameraMatrix_ =  node->getCameraMatrix();
    bool sim = true;
    if (sim) {
        cameraextrinsics_ = node->getCameracExtrinsics();
    }

    node->printCameraMatrices(cameraMatrix_);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}