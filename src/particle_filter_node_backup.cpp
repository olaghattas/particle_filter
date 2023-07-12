//
// Created by ola on 6/15/23.
//
#include "particle_filter.cpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <nlohmann/json.hpp>
#include <cv_bridge/cv_bridge.h>

#include <random>
#include <map>
#include <opencv2/opencv.hpp>
//#include "detection_msgs/msg/pose_msg.hpp"
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


//struct CameraIntrinsics {
//    geometry_msgs::msg::Vector3 translation;
//    geometry_msgs::msg::Quaternion rotation;
//};

class ParticleFilterNode : public rclcpp::Node {
private:

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cameraMatrix;
    Eigen::Matrix<double, 3, 4, Eigen::RowMajor> cameraProjectionMatrix;
    LandmarkObs observation; // Member variable to store the observation



public:
    ParticleFilterNode() : Node("particle_filter") {

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 10);

        // subscribe to point coordinate info to get intrinsic parameters
//        auto pose_sub = create_subscription<detection_msgs::msg::PoseMsg>(
//                "/coord_shoulder_joint_in_px", 1,
//                [this](const detection_msgs::msg::PoseMsg::SharedPtr msg) { PosePixCallback(msg); });

    }

    LandmarkObs getObservation() const {
        return observation;
    }

//    LandmarkObs PosePixCallback(const detection_msgs::msg::PoseMsg::SharedPtr &msg) {
//        LandmarkObs observation;
//        observation.name = msg->name;
//        observation.x = msg->coordinates.x;
//        observation.y = msg->coordinates.y;
//        return observation;
//    }

    void publish_particles(const std::vector<Particle> &particles) {       // Create a marker array message
        auto markerArrayMsg = std::make_shared<visualization_msgs::msg::MarkerArray>();
        // Populate the marker array with markers
        for (const auto &particle: particles) {
            // Create a marker message
            visualization_msgs::msg::Marker marker;

            // Set the marker properties
            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock()->now();
            marker.id = particle.id;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = particle.x;
            marker.pose.position.y = particle.y;
            marker.pose.position.z = particle.z;
            marker.pose.orientation.z = sin(particle.theta / 2.0);
            marker.pose.orientation.w = cos(particle.theta / 2.0);
            marker.scale.x = 0.05;  // Set the scale to make the arrow thinner
            marker.scale.y = 0.01;  // Set the scale to make the arrow thinner
            marker.scale.z = 0.01;  // Set the scale to make the arrow thinner
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            // Add the marker to the marker array
            markerArrayMsg->markers.push_back(marker);
        }
        // Publish the marker array
        publisher_->publish(*markerArrayMsg);

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

    geometry_msgs::msg::Transform get_extrinisics (const sensor_msgs::msg::Image::SharedPtr msg){
        geometry_msgs::msg::Transform cam_ext;
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
        cam_ext.rotation = rotation;
        return cam_ext;
    }

    geometry_msgs::msg::Transform get_pose_from_transform(const geometry_msgs::msg::TransformStamped::SharedPtr &msg){
        geometry_msgs::msg::Transform cam_ext;
        geometry_msgs::msg::Vector3 translation;
        cam_ext.translation = msg->transform.translation;
        geometry_msgs::msg::Quaternion rotation;
        cam_ext.rotation = msg->transform.rotation;
        return cam_ext;
    }

    std::map<std::string, geometry_msgs::msg::Transform> tfCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
    {
        std::map<std::string, geometry_msgs::msg::Transform> cameraextrinsics;
        // Check the child_frame_id
        std::string child_frame_id = msg->child_frame_id;
        if (child_frame_id == "tapo_camera_kitchen")
        {
            // Perform your desired actions here
            geometry_msgs::msg::Transform result_kitchen = get_pose_from_transform(msg);
            std::string kitchen = "kitchen";
            cameraextrinsics.insert(std::make_pair(kitchen, result_kitchen));

        }
        if (child_frame_id == "tapo_camera_dining")
        {
            // Perform your desired actions here
            geometry_msgs::msg::Transform result_dining = get_pose_from_transform(msg);
            std::string dine = "dining";
            cameraextrinsics.insert(std::make_pair(dine, result_dining));

        }
        if (child_frame_id == "tapo_camera_bedroom")
        {
            // Perform your desired actions here
            geometry_msgs::msg::Transform result_bedroom = get_pose_from_transform(msg);
            std::string bedroom = "bedroom";
            cameraextrinsics.insert(std::make_pair(bedroom, result_bedroom));

        }
        if (child_frame_id == "tapo_camera_living")
        {
            // Perform your desired actions here
            geometry_msgs::msg::Transform result_living = get_pose_from_transform(msg);
            std::string living = "living";
            cameraextrinsics.insert(std::make_pair(living, result_living));

        }
        return cameraextrinsics;
    }

};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParticleFilterNode>();
    std::map<std::string, geometry_msgs::msg::Transform> cameraextrinsics_;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cameraMatrix;

    // subscribe to camera info to get intrinsic parameters and save them in a map
    auto cameraInfoSub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/color/camera_info", 1,
            [node, &cameraMatrix](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                cameraMatrix = node->cameraInfoCallback(msg);
            });

    bool sim = true ;
    // to decide, from unity i have to subscribe to tf2, thinkng of kepng it the same or getiting extrininsc directly form checkerboard.
    if (!sim) {
        auto dining_camera_sub = node->create_subscription<sensor_msgs::msg::Image>(
                "/camera_dining_room/color/image_raw", 1,
                [node, &cameraextrinsics_](const sensor_msgs::msg::Image::SharedPtr msg) {
                    geometry_msgs::msg::Transform result_dining = node->get_extrinisics(msg);
                    std::string dine = "dining";
                    cameraextrinsics_.insert(std::make_pair(dine, result_dining));
                });

        auto living_camera_sub = node->create_subscription<sensor_msgs::msg::Image>(
                "/camera_living_room/color/image_raw", 1,
                [node, &cameraextrinsics_](const sensor_msgs::msg::Image::SharedPtr msg) {
                    geometry_msgs::msg::Transform result_living = node->get_extrinisics(msg);
                    std::string living = "living";
                    cameraextrinsics_.insert(std::make_pair(living, result_living));
                });

        auto bedroom_camera_sub = node->create_subscription<sensor_msgs::msg::Image>(
                "/camera_bedroom/color/image_raw", 1,
                [node, &cameraextrinsics_](const sensor_msgs::msg::Image::SharedPtr msg) {
                    geometry_msgs::msg::Transform result_bedroom = node->get_extrinisics(msg);
                    std::string bedroom = "bedroom";
                    cameraextrinsics_.insert(std::make_pair(bedroom, result_bedroom));
                });

        auto kitchen_camera_sub = node->create_subscription<sensor_msgs::msg::Image>(
                "/camera_kitchen/color/image_raw", 1,
                [node, &cameraextrinsics_](const sensor_msgs::msg::Image::SharedPtr msg) {
                    geometry_msgs::msg::Transform result_kitchen = node->get_extrinisics(msg);
                    std::string kitchen = "kitchen";
                    cameraextrinsics_.insert(std::make_pair(kitchen, result_kitchen));
                });
    }
    if (sim){

        auto tf_subscriber_ = node->create_subscription<geometry_msgs::msg::TransformStamped>(
                "/tf", 1, [node, &cameraextrinsics_](const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
            cameraextrinsics_ = node->tfCallback(msg);
        });
    }
    // Todo map observatiopn to camera and intrinsic extirinsics
    //    std::map<std::string, cv::Mat> cameraExtrinsics;
    //    cameraExtrinsics.insert(std::make_pair("dining", result_dining));


//    std::array<double, 4> sigma_pos = {0.3, 0.3, 0.3, 0.01};
//    double sigma_landmark[2] = {0.3, 0.3};
//
//    // noise generation
//    std::default_random_engine gen;
//
//    std::normal_distribution<double> N_obs_x(0, sigma_landmark[0]);
//    std::normal_distribution<double> N_obs_y(0, sigma_landmark[1]);
//
//    double n_x, n_y;
//
//    // Define the bounds based on the house
//    std::pair<double, double> x_bound = std::make_pair(0.0, 10.0);
//    std::pair<double, double> y_bound = std::make_pair(0.0, 20.0);
//    std::pair<double, double> z_bound = std::make_pair(0.0, 5.0);
//    std::pair<double, double> theta_bound = std::make_pair(-180.0, 180.0);
//
//    int num_particles = 100;
//
//    double velocity = 1.0;
//    double yaw_rate = 1.0;
//    bool running = true;
//
//    ParticleFilter particle_filter(num_particles);

//    while (running) {
//        auto beg = std::chrono::high_resolution_clock::now();
//        std::vector<LandmarkObs> observations;
//
//        LandmarkObs obs_ = node->getObservation();
//
//        // observation will always be from the same camera
//        std::string cam_name = obs_.name;
//        observations.push_back(obs_);
//
//
//        if (!particle_filter.initialized()) {
//
//            // Initialize the particle filter in a uniform distribution
//            particle_filter.init(x_bound, y_bound, z_bound, theta_bound);
//        } else {
//            // Predict the vehicle's next state (noiseless).
//            auto end = std::chrono::high_resolution_clock::now();
//            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - beg);
//            double delta_t = duration.count() / 1000000.0;
//            particle_filter.motion_model(delta_t, sigma_pos, velocity, yaw_rate);
//
//        }
//        // simulate the addition of noise to noiseless observation data.
//        std::vector<LandmarkObs> noisy_observations;
//        LandmarkObs obs;
//        // which is currently 1
//        for (int j = 0; j < observations.size(); ++j) {
//            n_x = N_obs_x(gen);
//            n_y = N_obs_y(gen);
//            obs = observations[j];
//            obs.x = obs.x + n_x;
//            obs.y = obs.y + n_y;
//            noisy_observations.push_back(obs);
//        }
//
//
//
//
//        Eigen::Quaterniond quaternion(cameraextrinsics[cam_name].quaternion.w, cameraextrinsics[cam_name].quaternion.x,
//                                      cameraextrinsics[cam_name].quaternion.y, cameraextrinsics[cam_name].quaternion.z);
//        Eigen::Matrix3d rotationMatrix = quaternion.normalized().toRotationMatrix();
//        Eigen::Vector3d translationVector(cameraextrinsics[cam_name].translation.x,
//                                          cameraextrinsics[cam_name].translation.y,
//                                          cameraextrinsics[cam_name].translation.z);
//        Eigen::Matrix4d extrinsicmatrix;
//        extrinsicmatrix.block<3, 3>(0, 0) = rotationMatrix;
//        extrinsicmatrix.block<3, 1>(0, 3) = translationVector;
//        extrinsicmatrix.row(3) << 0, 0, 0, 1;
//        // Update the weights and resample
//        particle_filter.updateWeights(sigma_landmark, noisy_observations, cameraMatrix, extrinsicmatrix);
//        particle_filter.resample();
//
//
//    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

//    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
//    {
//        // Check the child_frame_id
//        for (const auto& tf : msg->transforms) {
//
//            if (tf.child_frame_id == "tapo_camera_kitchen") {
//                // Perform your desired actions here
//                geometry_msgs::msg::Transform result_kitchen = tf.transform;
//                std::string kitchen = "kitchen";
//                cameraextrinsics.insert(std::make_pair(kitchen, result_kitchen));
//
//            }
//            if (tf.child_frame_id == "tapo_camera_dining") {
//                // Perform your desired actions here
//                geometry_msgs::msg::Transform result_dining = tf.transform;
//                std::string dine = "dining";
//                cameraextrinsics.insert(std::make_pair(dine, result_dining));
//
//            }
//            if (tf.child_frame_id == "tapo_camera_bedroom") {
//                // Perform your desired actions here
//                geometry_msgs::msg::Transform result_bedroom = tf.transform;
//                std::string bedroom = "bedroom";
//                cameraextrinsics.insert(std::make_pair(bedroom, result_bedroom));
//
//            }
//            if (tf.child_frame_id == "tapo_camera_living") {
//                // Perform your desired actions here
//                geometry_msgs::msg::Transform result_living = tf.transform;
//                std::string living = "living";
//                cameraextrinsics.insert(std::make_pair(living, result_living));
//
//            }
//        }
//    }