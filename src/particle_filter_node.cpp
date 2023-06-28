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
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include <nlohmann/json.hpp>


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

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    Eigen::Matrix3d cameraMatrix;

public:
    ParticleFilterNode() : Node("particle_filter") {
        // subscribe to camera info to get intrinsic parameters
        auto cameraInfoSub = create_subscription<sensor_msgs::msg::CameraInfo>(
                "/camera_info", 1,
                [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(msg); });

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 10);

    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        //# Camera intrinsic parameters (example values)
        //# Intrinsic camera matrix for the raw (distorted) images.
        //#     [fx  0 cx]
        //# K = [ 0 fy cy]
        //#     [ 0  0  1]
        cameraMatrix << msg->k[0], 0, msg->k[2],
                0, msg->k[4], msg->k[5],
                0, 0, 1;

    }

    void
    publish_particles(
            const std::vector<Particle> &particles) {       // Create a marker array message
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

    // reads from json file quaternion and position of the camera and outputs extrinsic parameters
    std::vector<Eigen::MatrixXd> Extrinsic() {

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
            double q1 = data.quatX;
            double q2 = data.quatY;
            double q3 = data.quatZ;
            double q0 = data.quatW;

            Eigen::MatrixXd r(4, 4);

            //            a nicer way to fill r matrix
            //            d1<< current_obs.d20, current_obs.d21, current_obs.d22, current_obs.d23, current_obs.d24,
            //                    current_obs.d25, current_obs.d26, current_obs.d27, current_obs.d28 ,current_obs.d29;
            //First row of the transformation matrix
            r(0, 0) = 2 * (q0 * q0 + q1 * q1) - 1;
            r(0, 1) = 2 * (q1 * q2 - q0 * q3);
            r(0, 2) = 2 * (q1 * q3 + q0 * q2);
            r(0, 3) = data.posX;

            // Second row of the transformation matrix
            r(1, 0) = 2 * (q1 * q2 + q0 * q3);
            r(1, 1) = 2 * (q0 * q0 + q2 * q2) - 1;
            r(1, 2) = 2 * (q2 * q3 - q0 * q1);
            r(1, 3) = data.posY;

            // Third row of the transformation matrix
            r(2, 0) = 2 * (q1 * q3 - q0 * q2);
            r(2, 1) = 2 * (q2 * q3 + q0 * q1);
            r(2, 2) = 2 * (q0 * q0 + q3 * q3) - 1;
            r(2, 3) = data.posZ;

            r(3, 0) = 0;
            r(3, 1) = 0;
            r(3, 2) = 0;
            r(3, 3) = 1;
            std::cout << r << std::endl;
            extrinsiclist.push_back(r);
        }
        return extrinsiclist;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParticleFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}