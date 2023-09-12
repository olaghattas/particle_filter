//
// Created by olagh on 9/11/23.
//


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "particle_filter.cpp"
#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>


#include <random>
#include <map>
#include <opencv2/opencv.hpp>
#include "detection_msgs/msg/pose_msg.hpp"
#include "detection_msgs/msg/door_status.hpp"
#include <array>
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"


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


class ParticleFilterNode : public rclcpp::Node {
private:

//    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
//    rclcpp::TimerBase::SharedPtr timer_;
    std::map<std::string, Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> cameraextrinsics;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cameraintrinsics;
    LandmarkObs observation; // Member variable to store the observation

//    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;


public:
    ParticleFilterNode() : Node("particle_filter") {

        auto func = [this]() -> void { cam_extrinsics_from_tf(); };
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), func);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


// TODO: change topic to jetson cameras
        auto cameraInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera/color/camera_info", 1,
                [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(msg); });

    }


    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr &msg) {
        // Access camera matrix values
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> K(msg->k.data());
        cameraintrinsics = K;
    }

    void cam_extrinsics_from_tf() {
        /// Test
//            std::map<std::string, std::string> map_cam_aptag;
        // map cameras to aptags ids
//            map_cam_aptag["dining"] = "aptagcam_extrinsics_from_tf_1";
//            map_cam_aptag["kitchen"] = "aptag_2";
//            map_cam_aptag["bedroom"] = "aptag_3";
//            map_cam_aptag["living"] = "aptag_4";
//        std::vector<std::string> cams{"dining", "kitchen", "bedroom", "livingroom", "hallway", "doorway"};
        std::vector<std::string> cams{"camera_color_optical_frame"};

        // Loop over the keys of map_cam_aptag using a range-based for loop
        for (const auto &cam: cams) {
            // Get transformation matrix from camera to aptag
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> t_aptag_to_cam = transform_tf("tag_18", cam);


            // Get transformation matrix from map to waptag
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> t_map_to_waptag = transform_tf("map", "aptag_18");

            // Get transformation matrix from map to aptag
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> t_map_to_cam = t_map_to_waptag * t_aptag_to_cam;

//            cameraextrinsics.insert(std::make_pair(cam, t_map_to_cam));
            cameraextrinsics.insert(std::make_pair("name", t_map_to_cam));
        }
    }

    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> transform_tf(std::string toFrame, std::string fromFrame) {

        try {
            // get the geometry transform frames
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                    toFrame, fromFrame,
                    tf2::TimePoint(), std::chrono::milliseconds(50));

            geometry_msgs::msg::Transform transform_ = t.transform;

            // turn geometry transform to 4x4 matrix
            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> transform = transform_geometry_to_matrix(transform_);
            RCLCPP_INFO(this->get_logger(), "transform %s to %s", fromFrame.c_str(), toFrame.c_str());

            return transform;

        }
        catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(
                    this->get_logger(), "Could not transform %s to %s: %s",
                    fromFrame.c_str(), toFrame.c_str(), ex.what());
//            return;
        }
    }

    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> get_cam_intrinsic_matrix() {
        return cameraintrinsics;
    }

    std::map<std::string, Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> get_cam_extrinsic_matrix() {
        return cameraextrinsics;
    }

    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> transform_geometry_to_matrix(geometry_msgs::msg::Transform transform) {
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> extrinsicmatrix;
        Eigen::Quaterniond quaternion(transform.rotation.w,
                                      transform.rotation.x,
                                      transform.rotation.y,
                                      transform.rotation.z);
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> rotationMatrix = quaternion.normalized().toRotationMatrix();
        Eigen::Vector3d translationVector(transform.translation.x,
                                          transform.translation.y,
                                          transform.translation.z);

        extrinsicmatrix.block<3, 3>(0, 0) = rotationMatrix;
        extrinsicmatrix.block<3, 1>(0, 3) = translationVector;
        extrinsicmatrix.row(3) << 0, 0, 0, 1;
        return extrinsicmatrix;
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> matrix1;
    matrix1 << 0.945483, 0.0750775, 0.3169, -3.75478,
            -0.324886, 0.149886, 0.933801, -5.05347,
            0.0226086, -0.985849, 0.166106, -0.748818,
            0, 0, 0, 1;
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> matrix2;
    matrix2 << 0, 1, 0, -2.692,
            -1, 0, 0, -2.276,
            0, 0, 1, -0.416,
            0, 0, 0, 1;

    // Compute the inverse of matrix2
//    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> inverse_matrix1 = matrix1.inverse();

    // Compute the dot product of matrix2 and its inverse
//    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> dot_product_result = matrix2 * inverse_matrix1;

    // Compute the dot product of matrix2 and its inverse
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> dot_product_result = matrix2 * matrix1;

    // Print the result
    std::cout << "Dot Product Result:" << std::endl << dot_product_result << std::endl;

    Eigen::Matrix4d matrix3;
    matrix3 << 0.945483, 0.0750775, 0.3169, -3.75478,
            -0.324886, 0.149886, 0.933801, -5.05347,
            0.0226086, -0.985849, 0.166106, -0.748818,
            0, 0, 0, 1;
    Eigen::Matrix4d matrix4;
    matrix4 << 0, 1, 0, -2.692,
            -1, 0, 0, -2.276,
            0, 0, 1, -0.416,
            0, 0, 0, 1;

    // Compute the dot product of matrix2 and its inverse
    Eigen::Matrix4d dot_product_result_2 = matrix3 * matrix4;
    std::cout << "Dot Product Result:" << std::endl << dot_product_result_2 << std::endl;
//    auto node = std::make_shared<ParticleFilterNode>();
//
//    auto tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
//    std::map<std::string, Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> camera_extrinsics;
//    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_intrinsics;
//
//
//    bool not_initialized = true;
//    while (rclcpp::ok()) {
//        if (not_initialized) {
//            if (camera_intrinsics.size() == 0) {
//                camera_intrinsics = node->get_cam_intrinsic_matrix();
//            }
//            if (camera_extrinsics.size() == 0) {
//                camera_extrinsics = node->get_cam_extrinsic_matrix();
//            }
//            if (camera_intrinsics.size() != 0 && camera_extrinsics.size() != 0) {
//                not_initialized = false;
//                for (const auto &entry: camera_extrinsics) {
//                    const std::string &key = entry.first;
//                    const Eigen::Matrix4d &matrix = entry.second;
//
//                    std::cout << "Key: " << key << "\n";
//                    std::cout << "Matrix:\n" << matrix << "\n";
//                }
//            }
//        }
    return 0;
}
//}