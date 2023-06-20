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
};


//    Rotation Matrix:
//            Convert the XYZ rotation values from Unity into a rotation matrix that represents the camera's orientation in the world coordinate system. The conversion depends on the rotation order and conventions used by Unity. Common rotation order conventions are XYZ, YXZ, ZYX, etc. You need to determine the specific order used in Unity.
//
//    For example, if the rotation order is XYZ, you can create the rotation matrix as follows:
//    Eigen::
//    Eigen::AngleAxisd rotation_x(rotation_x_value, Eigen::Vector3d::UnitX());
//    Eigen::AngleAxisd rotation_y(rotation_y_value, Eigen::Vector3d::UnitY());
//    Eigen::AngleAxisd rotation_z(rotation_z_value, Eigen::Vector3d::UnitZ());
//
//    Eigen::Matrix3d rotationMatrix = (rotation_z * rotation_y * rotation_x).toRotationMatrix();


    int main(int argc, char **argv) {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<ParticleFilterNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }