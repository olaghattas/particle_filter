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

//    std::vector<bool> door_status_;
    bool door_outdoor;
    bool door_livingroom;
    bool door_bedroom;
    bool door_bathroom;


public:
    ParticleFilterNode() : Node("particle_filter") {

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//         subscribe to point coordinate info to get intrinsic parameters
//        auto pose_sub_k = create_subscription<detection_msgs::msg::PoseMsg>(
//                "/coord_shoulder_joint_in_px_kitchen", 1,
//                [this](const detection_msgs::msg::PoseMsg::SharedPtr msg) { PosePixCallback_kitchen(msg); });
        auto pose_sub_din = create_subscription<detection_msgs::msg::PoseMsg>(
                "/coord_shoulder_joint_in_px_dining", 1,
                [this](const detection_msgs::msg::PoseMsg::SharedPtr msg) { PosePixCallback_dining(msg); });
//        auto pose_sub_lr = create_subscription<detection_msgs::msg::PoseMsg>(
//                "/coord_shoulder_joint_in_px_livingroom", 1,
//                [this](const detection_msgs::msg::PoseMsg::SharedPtr msg) { PosePixCallback_livingroom(msg); });
//        auto pose_sub_hw = create_subscription<detection_msgs::msg::PoseMsg>(
//                "/coord_shoulder_joint_in_px_hallway", 1,
//                [this](const detection_msgs::msg::PoseMsg::SharedPtr msg) { PosePixCallback_hallway(msg); });
//        auto pose_sub_dw = create_subscription<detection_msgs::msg::PoseMsg>(
//                "/coord_shoulder_joint_in_px_doorway", 1,
//                [this](const detection_msgs::msg::PoseMsg::SharedPtr msg) { PosePixCallback_doorway(msg); });

        auto door_outdoor_sub = create_subscription<detection_msgs::msg::DoorStatus>(
                "/smartthings_sensors_door_outdoor", 10,
                [this](const detection_msgs::msg::DoorStatus::SharedPtr msg) { DoorOutdoorCallback(msg); });
        auto door_livingroom_sub = create_subscription<detection_msgs::msg::DoorStatus>(
                        "/smartthings_sensors_door_livingroom", 10,
                        [this](const detection_msgs::msg::DoorStatus::SharedPtr msg) { DoorLivingroomCallback(msg); });
        auto door_bedroom_sub = create_subscription<detection_msgs::msg::DoorStatus>(
                        "/smartthings_sensors_door_bedroom", 10,
                        [this](const detection_msgs::msg::DoorStatus::SharedPtr msg) { DoorBedroomCallback(msg); });
        auto door_bathroom_sub = create_subscription<detection_msgs::msg::DoorStatus>(
                        "/smartthings_sensors_door_bathroom", 10,
                        [this](const detection_msgs::msg::DoorStatus::SharedPtr msg) { DoorBathroomCallback(msg); });

        auto cameraInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera/color/camera_info", 1,
                [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(msg); });

    }

    void DoorOutdoorCallback(const detection_msgs::msg::DoorStatus::SharedPtr &msg) {
        door_outdoor = msg->open;
    }
    void DoorLivingroomCallback(const detection_msgs::msg::DoorStatus::SharedPtr &msg) {
        door_livingroom = msg->open;
    }
    void DoorBedroomCallback(const detection_msgs::msg::DoorStatus::SharedPtr &msg) {
        door_bedroom = msg->open;
    }
    void DoorBathroomCallback(const detection_msgs::msg::DoorStatus::SharedPtr &msg) {
        door_bathroom = msg->open;
    }

    std::vector<bool> getdoorstatus() {
        return {door_outdoor, door_livingroom, door_bedroom, door_bathroom};
    }

    LandmarkObs getObservation() const {
        return observation;
    }

//    LandmarkObs PosePixCallback_kitchen(const detection_msgs::msg::PoseMsg::SharedPtr &msg) {
////        LandmarkObs observation;
//        observation.name = msg->name;
//        observation.x = msg->pixel_coordinate_x;
//        observation.y = msg->pixel_coordinate_y;
//        return observation;
//    }
    LandmarkObs PosePixCallback_dining(const detection_msgs::msg::PoseMsg::SharedPtr &msg) {
//        LandmarkObs observation;
        observation.name = msg->name;
        observation.x = msg->pixel_coordinate_x;
        observation.y = msg->pixel_coordinate_y;
        return observation;
    }
//    LandmarkObs PosePixCallback_livingroom(const detection_msgs::msg::PoseMsg::SharedPtr &msg) {
////        LandmarkObs observation;
//        observation.name = msg->name;
//        observation.x = msg->pixel_coordinate_x;
//        observation.y = msg->pixel_coordinate_y;
//        return observation;
//    }
//    LandmarkObs PosePixCallback_hallway(const detection_msgs::msg::PoseMsg::SharedPtr &msg) {
////        LandmarkObs observation;
//        observation.name = msg->name;
//        observation.x = msg->pixel_coordinate_x;
//        observation.y = msg->pixel_coordinate_y;
//        return observation;
//    }
//    LandmarkObs PosePixCallback_doorway(const detection_msgs::msg::PoseMsg::SharedPtr &msg) {
////        LandmarkObs observation;
//        observation.name = msg->name;
//        observation.x = msg->pixel_coordinate_x;
//        observation.y = msg->pixel_coordinate_y;
//        return observation;
//    }


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
        cameraintrinsics = K;
    }

    void cam_extrinsics_from_tf() {
        /// Test
//            std::map<std::string, std::string> map_cam_aptag;
        // map cameras to aptags ids
//            map_cam_aptag["dining"] = "aptag_1";
//            map_cam_aptag["kitchen"] = "aptag_2";
//            map_cam_aptag["bedroom"] = "aptag_3";
//            map_cam_aptag["living"] = "aptag_4";
        std::vector<std::string> cams{"dining", "kitchen", "bedroom", "livingroom", "hallway", "doorway"};
        // Loop over the keys of map_cam_aptag using a range-based for loop
        for (const auto &cam: cams) {
            // Get transformation matrix from camera to aptag
            Eigen::Matrix4d t_map_to_cam = transform_tf(cam);
            // Get transformation matrix from map to aptag

            cameraextrinsics.insert(std::make_pair(cam, t_map_to_cam));
        }
    }

    Eigen::Matrix4d transform_tf(std::string toFrame) {
        std::string fromFrame = "map";
        try {
            // get the geometry transform frames
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                    toFrame, fromFrame,
                    tf2::TimePoint(), std::chrono::milliseconds(50));

            geometry_msgs::msg::Transform transform_ = t.transform;

            // turn geometry transform to 4x4 matrix
            Eigen::Matrix4d transform = transform_geometry_to_matrix(transform_);
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

    std::map<std::string, Eigen::Matrix4d> get_cam_extrinsic_matrix() {
        return cameraextrinsics;
    }

    Eigen::Matrix4d transform_geometry_to_matrix(geometry_msgs::msg::Transform transform) {
        Eigen::Matrix4d extrinsicmatrix;
        Eigen::Quaterniond quaternion(transform.rotation.w,
                                      transform.rotation.x,
                                      transform.rotation.y,
                                      transform.rotation.z);
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

    auto tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    std::map<std::string, Eigen::Matrix4d> camera_extrinsics;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_intrinsics;

    tcnn::cpp::Module *network = nullptr;

    // Todo map observation to camera intrinsic and extrinsics
    //    std::map<std::string, cv::Mat> cameraExtrinsics;
    //    cameraExtrinsics.insert(std::make_pair("dining", result_dining));

    bool not_initialized = true;
    while (rclcpp::ok()) {
        if (not_initialized) {
            if (camera_intrinsics.size() == 0) {
                camera_intrinsics = node->get_cam_intrinsic_matrix();
            }
            if (camera_extrinsics.size() == 0) {
                camera_extrinsics = node->get_cam_extrinsic_matrix();
            }
            if (camera_intrinsics.size() != 0 && camera_extrinsics.size() != 0) {
                not_initialized = false;
            }
        } else {
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
            std::pair<double, double> z_bound = std::make_pair(-1.0, 1.0);
            std::pair<double, double> theta_bound = std::make_pair(-180.0, 180.0);

            int num_particles = 128*6; // has to be multiple of 128

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

                    /// todo fix
                    double delta_t = duration.count() / 1000000.0;
                    delta_t = 0.1; // fr debug

//
//                    if (!network){
//                        // train the model
//                        network = check_collision_training(directoryPath, 3);
//                    }

                    particle_filter.motion_model(delta_t, sigma_pos, velocity, yaw_rate, node->getdoorstatus());
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
                particle_filter.updateWeights(sigma_landmark, noisy_observations, camera_intrinsics,
                                              camera_extrinsics[cam_name]);
                particle_filter.resample();

                // Calculate and output the average weighted error of the particle filter over all time steps so far.
                std::vector <Particle> particles = particle_filter.particles;
                int num_particles = particles.size();
                double highest_weight = 0.0;

                Particle best_particle;
                for (int i = 0; i < num_particles; ++i) {
                    if (particles[i].weight > highest_weight) {
                        highest_weight = particles[i].weight;
                        best_particle = particles[i];
                    }
                }

                // Fill in the message
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = rclcpp::Clock().now();
                t.header.frame_id = "map";
                /// should be whatever the code is expecting the name to be
                t.child_frame_id = "nathan";
                t.transform.translation.x = best_particle.x;
                t.transform.translation.y = best_particle.y;
                t.transform.translation.z = best_particle.z;
                t.transform.rotation.x = 0;
                t.transform.rotation.y = 0;
                t.transform.rotation.z = sin(best_particle.theta / 2.0);
                t.transform.rotation.w = cos(best_particle.theta / 2.0);
                tf_broadcaster_->sendTransform(t);
            }
        }
        rclcpp::spin_some(node);

    }
    rclcpp::shutdown();
    return 0;
}