//
// Created by ola on 6/15/23.
//

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "particle_filter.cpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
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

#include <random>
#include <map>
#include <opencv2/opencv.hpp>
#include "detection_msgs/msg/pose_msg.hpp"
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


//struct CameraIntrinsics {
//    geometry_msgs::msg::Vector3 translation;
//    geometry_msgs::msg::Quaternion rotation;
//};

class ParticleFilterNode : public rclcpp::Node {
private:

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
//    rclcpp::TimerBase::SharedPtr timer_;
    std::map<std::string, Eigen::Matrix4d> cameraextrinsics;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cameraintrinsics;
    LandmarkObs observation; // Member variable to store the observation

    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;



public:
    ParticleFilterNode() : Node("particle_filter") {

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//         subscribe to point coordinate info to get intrinsic parameters
        auto pose_sub = create_subscription<detection_msgs::msg::PoseMsg>(
                "/coord_shoulder_joint_in_px", 1,
                [this](const detection_msgs::msg::PoseMsg::SharedPtr msg) { PosePixCallback(msg); });

        auto cameraInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera/color/camera_info", 1,
                [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(msg); });

        //fill the cam extrinsic map
        // TODO: not call it all the time only when needed, need to check how it is done in C++
        cam_extrinsics_from_tf();

    }

    LandmarkObs getObservation() const {
        return observation;
    }

    LandmarkObs PosePixCallback(const detection_msgs::msg::PoseMsg::SharedPtr &msg) {
        LandmarkObs observation;
        observation.name = msg->name;
        observation.x = msg->pixel_coordinate_x;
        observation.y = msg->pixel_coordinate_y;
        return observation;
    }

    void publish_particles(std::vector<Particle> &particles) {       // Create a marker array message
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

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr &msg) {
        // Access camera matrix values
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> K(msg->k.data());
        cameraMatrix = K;
    }

    void cam_extrinsics_from_yaml() {
        std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("particle_filter");
        auto file_path = pkg_dir / "config" / "transformation_matrix.yaml";

        // Load YAML file
        YAML::Node yaml_data = YAML::LoadFile(file_path);



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
            std::string camera = transformation["camera"].as<std::string>();
        std::string toFrame = "aptag_" + matrix_id;

        try {
            // get the geometry transform between aptag and camera
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                    toFrame, camera,
                    tf2::TimePoint(),std::chrono::milliseconds(50));

            geometry_msgs::msg::Transform transform_aptag_in_cam_geom = t.transform;

            std::string cam_loc;
            size_t lastSpace = camera.find_last_of(' ');
            if (lastSpace != std::string::npos) {
                cam_loc = camera.substr(lastSpace + 1);
            } else {
                cam_loc = camera;
            }
            // turn geometry transform to 4x4 matrix
            Eigen::Matrix4d transform_aptag_in_cam = transform_geometry_to_matrix(transform_aptag_in_cam_geom);

            //multiply the transform of the aptag in the world with the inverse of the transform matrix
            // TODO: check if this is matrix or element wise multiplication
            Eigen::Matrix4d transform_ = matrix * transform_aptag_in_cam.inverse();
            cameraextrinsics.insert(std::make_pair(cam_loc, transform_));

        } catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(
                    this->get_logger(), "Could not transform %s to %s: %s",
                    toFrame.c_str(), camera.c_str(), ex.what());
            return;
        }

        }
    }

    void cam_extrinsics_from_tf( std::string fromFrame, std::string toFrame) {
            // TODO: instead of manually mapping camera to aptag, use a Yaml file
            std::map<std::string, std::string> map_cam_aptag;
            // map cameras to aptags ids
            map_cam_aptag["dining"] = "aptag_1";
            map_cam_aptag["kitchen"] = "aptag_2";
            map_cam_aptag["bedroom"] = "aptag_3";
            map_cam_aptag["living"] = "aptag_4";

            // Loop over the keys of map_cam_aptag using a range-based for loop
            for (const auto& entry : map_cam_aptag) {
                // Get transformation matrix from camera to aptag
                Eigen::Matrix4d t_camera_to_aptag = transform_tf(entry.first, entry.second) ;
                // Get transformation matrix from map to aptag
                Eigen::Matrix4d t_map_to_aptag = transform_tf("map", entry.second) ;

                //multiply the transform of the aptag in the world with the inverse of the transform matrix
                // TODO: check if this is matrix or element wise multiplication
                Eigen::Matrix4d transform_map_to_cam = t_map_to_aptag * t_camera_to_aptag.inverse();
                cameraextrinsics.insert(std::make_pair(entry.first, transform_));
            }
    }

    Eigen::Matrix4d transform_tf( std::string fromFrame, std::string toFrame) {
        try {
            // get the geometry transform frames
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                    toFrame, fromFrame,
                    tf2::TimePoint(),std::chrono::milliseconds(50));

            geometry_msgs::msg::Transform transform_= t.transform;

            // turn geometry transform to 4x4 matrix
            Eigen::Matrix4d transform = transform_geometry_to_matrix(transform_);
            return transform

        }
        catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(
                    this->get_logger(), "Could not transform %s to %s: %s",
                    toFrame.c_str(), fromFrame.c_str(), ex.what());
            return;
        }


    }

    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> get_cam_intrinsic_matrix() {
        return cameraintrinsics;
    }

    std::map<std::string, Eigen::Matrix4d> get_cam_extrinsic_matrix() {
        return cameraextrinsics;
    }

    Eigen::Matrix4d transform_geometry_to_matrix(geometry_msgs::msg::Transform transform){
        Eigen::Matrix4d extrinsicmatrix;
//        Eigen::Quaterniond quaternion(transform.rotation.w,
//                                      transform.rotation.x,
//                                      transform.rotation.y,
//                                      transform.rotation.z);
        Eigen::Matrix3d rotationMatrix = quaternion.normalized().toRotationMatrix();
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
    auto node = std::make_shared<ParticleFilterNode>();
    std::map<std::string, Eigen::Matrix4d> camera_extrinsics;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_intrinsics;

    // Todo map observation to camera and intrinsic extrinsics
    //    std::map<std::string, cv::Mat> cameraExtrinsics;
    //    cameraExtrinsics.insert(std::make_pair("dining", result_dining));

    bool not_initialized = true;
    while (rclcpp::ok()) {
        if (not_initialized){
            if (camera_intrinsics.size() == 0) {
                camera_intrinsics = node->get_cam_intrinsic_matrix();
            }
            if (camera_extrinsics.size() == 0) {
                camera_extrinsics = node->get_cam_extrinsic_matrix();
            }
            if (camera_intrinsics.size() != 0 && camera_extrinsics.size() != 0) {
                not_initialized = false;
            }
        }
        else {
            std::array<double, 4> sigma_pos = {0.3, 0.3, 0.3, 0.01};

            double sigma_landmark[2] = {0.3, 0.3};

            // noise generation
            std::default_random_engine gen;

            std::normal_distribution<double> N_obs_x(0, sigma_landmark[0]);
            std::normal_distribution<double> N_obs_y(0, sigma_landmark[1]);

            double n_x, n_y;

            // Define the bounds based on the house
            std::pair<double, double> x_bound = std::make_pair(-5, 5.0);
            std::pair<double, double> y_bound = std::make_pair(-7.0, 7.0);
            std::pair<double, double> z_bound = std::make_pair(-2.0, 2.0);
            std::pair<double, double> theta_bound = std::make_pair(-180.0, 180.0);

            int num_particles = 500;

            double velocity = 0.01;
            double yaw_rate = 1.0;
            bool running = true;

            ParticleFilter particle_filter(num_particles);

            while (running) {
                auto beg = std::chrono::high_resolution_clock::now();
                std::vector<LandmarkObs> observations;

                LandmarkObs obs_ = node->getObservation();

                // observation will always be from the same camera
                std::string cam_name = obs_.name;
                observations.push_back(obs_);


                if (!particle_filter.initialized()) {

                    // Initialize the particle filter in a uniform distribution
                    particle_filter.init(x_bound, y_bound, z_bound, theta_bound);
                    node->publish_particles(particle_filter.particles);
                } else {
                    // Predict the vehicle's next state (noiseless).
                    auto end = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - beg);
                    double delta_t = duration.count() / 1000000.0;

                    delta_t = 0.1; // fr debug
                    particle_filter.motion_model(delta_t, sigma_pos, velocity, yaw_rate);
                    node->publish_particles(particle_filter.particles);

                }

                node->publish_particles(particle_filter.particles);
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

                // Update the weights and resample
                particle_filter.updateWeights(sigma_landmark, noisy_observations, camera_intrinsics, camera_extrinsics[cam_name]);
                particle_filter.resample();
            }
        }
        rclcpp::spin_some(node);

    }
    rclcpp::shutdown();
    return 0;
}