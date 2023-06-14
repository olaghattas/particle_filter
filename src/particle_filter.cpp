//
// Created by ola on 6/14/23.
//

#include "particle_filter/particle_filter.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>

#include "std_msgs/msg/string.hpp"


class ParticleFilter : public rclcpp::Node //creates a node class by inheriting from node
{
public:
    ParticleFilter()
    : Node("particle_filter")
    {
        // publish for the marker array
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker_array_topic", 10);

        // timer to periodically publish the array
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ParticleFilter::publishMarkerArray, this));
    }

private:
    void publishMarkerArray()
    {
        // Create a marker array message
        auto markerArrayMsg = std::make_shared<visualization_msgs::msg::MarkerArray>();

        // Populate the marker array with markers
        for (int i = 0; i < 5; ++i)
        {
            // Create a marker message
            visualization_msgs::msg::Marker marker;

            // Set the marker properties
            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock()->now();
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = i * 0.1;
            marker.pose.position.y = i * 0.2;
            marker.pose.position.z = i * 0.3;
            marker.pose.orientation.w = 1.0;
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

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParticleFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}