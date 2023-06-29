//
// Created by ola on 6/15/23.
//
#include "particle_filter/particle_filter.h"
//#include "particle_filter.cpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <nlohmann/json.hpp>
#include "colmap_srv/srv/image_transform.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <random>
#include <map>
#include <opencv2/opencv.hpp>
#include "detection_msgs/msg/pose_msg.hpp"
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
    rclcpp::Client<colmap_srv::srv::ImageTransform>::SharedPtr client_;


public:
    ParticleFilterNode() : Node("particle_filter") {

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 10);
        client_ = create_client<colmap_srv::srv::ImageTransform>(
                "colmap_service");
        // subscribe to point coordinate info to get intrinsic parameters

        auto pose_sub = create_subscription<detection_msgs::msg::PoseMsg>(
                "/coord_shoulder_joint_in_px", 1,
                [this](const detection_msgs::msg::PoseMsg::SharedPtr msg) { PosePixCallback(msg); });

    }

    LandmarkObs getObservation() const {
        return observation;
    }

    LandmarkObs PosePixCallback(const detection_msgs::msg::PoseMsg::SharedPtr &msg) {
        LandmarkObs observation;
        observation.name = msg->name;
        observation.x = msg->coordinates.x;
        observation.y = msg->coordinates.y;
        return observation;
    }

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

    Pose colmap_image_registering_client(const sensor_msgs::msg::Image::SharedPtr &msg, const std::string image_name) {

        //    std::string pkgpath = ament_index_cpp::get_package_share_directory("colmap_localization");
        // used this because the ament refers to the clmap_localization in the install directory
        std::string pkgpath = "/home/ola/smart_home/src/smart-home/external/ros2_colmap/colmap_localization/image_registering";

        std::string img_path = pkgpath + "/test_images";
        std::string input_path = pkgpath + "/sparse/0";
        std::string db_path = pkgpath + "/database.db";
        std::string img_list = pkgpath + "/image-list.txt";

        // save the images in a folder
        SaveImage(msg, img_path + image_name);

        // save the name of the new image in the list of images
        // Open the file in output mode and clear its contents
        std::ofstream file(img_list, std::ios::out | std::ios::trunc);

        if (!file) {
            std::cout << "Failed to open the file." << std::endl;
        }

        // Write new content to the file
        file << image_name + "\n";
        // Close the file
        file.close();


        auto request = std::make_shared<colmap_srv::srv::ImageTransform::Request>();
        request->path_to_new_images = img_path;
        request->path_to_images_list = img_list;
        request->path_to_db = db_path;
        request->path_to_reconstructed_images = pkgpath + "/image_registering/sparse/0";;

        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");

            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result = client_->async_send_request(request);

        std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("colmap_service");
        if (rclcpp::spin_until_future_complete(node, result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Success");

            Pose cam_ext;
            geometry_msgs::msg::Vector3 translation = result.get()->translation;
            cam_ext.translation = translation;
            geometry_msgs::msg::Quaternion rotation = result.get()->rotation;
            cam_ext.quaternion = rotation;
            return cam_ext;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
        }
    }

    void SaveImage(const sensor_msgs::msg::Image::SharedPtr &msg, std::string path) {
        try {
//            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//            cv::Mat image = cv_ptr->image;
            // Convert the sensor_msgs::Image to a cv::Mat
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

            // Specify the file path and name to save the image
            std::string file_path = path;
            // Save the image to the specified path
            cv::imwrite(file_path, image);

            std::cout << "Image saved to: " << file_path << std::endl;
        }
        catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParticleFilterNode>();
    std::map<std::string, Pose> cameraextrinsics;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cameraMatrix;

    // subscribe to camera info to get intrinsic parameters and save them in a map
    auto cameraInfoSub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", 1,
            [node, &cameraMatrix](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                cameraMatrix = node->cameraInfoCallback(msg);
            });

    auto dining_camera_sub = node->create_subscription<sensor_msgs::msg::Image>(
            "/camera_dining_room/color/image_raw", 1,
            [node, &cameraextrinsics](const sensor_msgs::msg::Image::SharedPtr msg) {
                Pose result_dining = node->colmap_image_registering_client(msg, "dining.jpg");
                std::string dine = "dining";
                cameraextrinsics.insert(std::make_pair(dine, result_dining));
            });

    auto living_camera_sub = node->create_subscription<sensor_msgs::msg::Image>(
            "/camera_living_room/color/image_raw", 1,
            [node, &cameraextrinsics](const sensor_msgs::msg::Image::SharedPtr msg) {
                Pose result_living = node->colmap_image_registering_client(msg, "living.jpg");
                std::string living = "living";
                cameraextrinsics.insert(std::make_pair(living, result_living));
            });

    auto bedroom_camera_sub = node->create_subscription<sensor_msgs::msg::Image>(
            "/camera_bedroom/color/image_raw", 1,
            [node, &cameraextrinsics](const sensor_msgs::msg::Image::SharedPtr msg) {
                Pose result_bedroom = node->colmap_image_registering_client(msg, "bedroom.jpg");
                std::string bedroom = "bedroom";
                cameraextrinsics.insert(std::make_pair(bedroom, result_bedroom));
            });

    auto kitchen_camera_sub = node->create_subscription<sensor_msgs::msg::Image>(
            "/camera_kitchen/color/image_raw", 1,
            [node, &cameraextrinsics](const sensor_msgs::msg::Image::SharedPtr msg) {
                Pose result_kitchen = node->colmap_image_registering_client(msg, "kitchen.jpg");
                std::string kitchen = "kitchen";
                cameraextrinsics.insert(std::make_pair(kitchen, result_kitchen));
            });

    // Todo map observatiopn to camera and intrinsic extirinsics
//    std::map<std::string, cv::Mat> cameraExtrinsics;
//    cameraExtrinsics.insert(std::make_pair("dining", result_dining));



    std::array<double, 4> sigma_pos = {0.3, 0.3, 0.3, 0.01};
    double sigma_landmark[2] = {0.3, 0.3};

    // noise generation
    std::default_random_engine gen;
    std::normal_distribution<double> N_x_init(0, sigma_pos[0]);
    std::normal_distribution<double> N_y_init(0, sigma_pos[1]);
    std::normal_distribution<double> N_z_init(0, sigma_pos[2]);
    std::normal_distribution<double> N_theta_init(0, sigma_pos[3]);

    std::normal_distribution<double> N_obs_x(0, sigma_landmark[0]);
    std::normal_distribution<double> N_obs_y(0, sigma_landmark[1]);

    double n_x, n_y, n_z, n_theta, n_range, n_heading;

    // Define the bounds based on the house
    std::pair<double, double> x_bound = std::make_pair(0.0, 10.0);
    std::pair<double, double> y_bound = std::make_pair(0.0, 20.0);
    std::pair<double, double> z_bound = std::make_pair(0.0, 5.0);
    std::pair<double, double> theta_bound = std::make_pair(-180.0, 180.0);

    int num_particles = 100;

    double velocity = 1.0;
    double yaw_rate = 1.0;
    bool running = true;

    ParticleFilter particle_filter(num_particles);

    while (running) {
        auto beg = std::chrono::high_resolution_clock::now();
        std::vector<LandmarkObs> observations;
        LandmarkObs obs_ = node->getObservation();
        observations.push_back(obs_);
        // TODO :  get the observation from the subscription callback
//        observations.push_back(obs);

        if (!particle_filter.initialized()) {
            n_x = N_x_init(gen);
            n_y = N_y_init(gen);
            n_theta = N_theta_init(gen);
            // Initialize the particle filter
            particle_filter.init(x_bound, y_bound, z_bound, theta_bound);
        } else {
            // Predict the vehicle's next state (noiseless).
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - beg);
            double delta_t = duration.count() / 1000000.0;
            particle_filter.motion_model(delta_t, sigma_pos, velocity, yaw_rate);

        }
        // simulate the addition of noise to noiseless observation data.
        std::vector<LandmarkObs> noisy_observations;
        LandmarkObs obs;
        // which is currently 1
        for (int j = 0; j < observations.size(); ++j) {
            n_x = N_obs_x(gen);
            n_y = N_obs_y(gen);
            obs = observations[j];
            obs.x = obs.x + n_x;
            obs.y = obs.y + n_y;
            noisy_observations.push_back(obs);
        }


        std::string cam_name = "dining";

        Eigen::Quaterniond quaternion(cameraextrinsics[cam_name].quaternion.w, cameraextrinsics[cam_name].quaternion.x,
                                      cameraextrinsics[cam_name].quaternion.y, cameraextrinsics[cam_name].quaternion.z);
        Eigen::Matrix3d rotationMatrix = quaternion.normalized().toRotationMatrix();
        Eigen::Vector3d translationVector(cameraextrinsics[cam_name].translation.x,
                                          cameraextrinsics[cam_name].translation.y,
                                          cameraextrinsics[cam_name].translation.z);
        Eigen::Matrix4d extrinsicmatrix;
        extrinsicmatrix.block<3, 3>(0, 0) = rotationMatrix;
        extrinsicmatrix.block<3, 1>(0, 3) = translationVector;
        extrinsicmatrix.row(3) << 0, 0, 0, 1;
        // Update the weights and resample
        particle_filter.updateWeights(sigma_landmark, noisy_observations, cameraMatrix, extrinsicmatrix);
        particle_filter.resample();


    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}