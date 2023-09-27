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


        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

// TODO: change topic to jetson cameras

        auto pose_sub_lr = create_subscription<detection_msgs::msg::PoseMsg>(
                "/coord_shoulder_joint_in_px_livingroom", 1,
                [this](const detection_msgs::msg::PoseMsg::SharedPtr msg) { PosePixCallback_livingroom(msg); });
//

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

    LandmarkObs getObservation() const {
        return observation;
    }
    LandmarkObs PosePixCallback_livingroom(const detection_msgs::msg::PoseMsg::SharedPtr &msg) {
//        LandmarkObs observation;
        observation.name = msg->name;
        observation.x = msg->pixel_coordinate_x;
        observation.y = msg->pixel_coordinate_y;
        return observation;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParticleFilterNode>();
    auto tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);


    bool not_initialized = true;
    while (rclcpp::ok()) {

        LandmarkObs obs_ = node->getObservation();
        Eigen::Vector4d point_wrt_camera_optical;
        point_wrt_camera_optical << 3,-1,2,0; // obs_->x, obs_->y, 0.0, 1;

        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Tmap_aptag ;
        Tmap_aptag  << 0, 1, 0, -2.692,
                -1, 0, 0, -2.276,
                0, 0, 1, -0.416,
                0, 0, 0, 1; // = node->transform_tf("map", "aptag_18") ;
        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Taptag_cam;   // = node->transform_tf("tag_18", "camera_color_optical_frame");
        Taptag_cam << 0.945483, 0.0750775, 0.3169, -3.75478,
                -0.324886, 0.149886, 0.933801, -5.05347,
                0.0226086, -0.985849, 0.166106, -0.748818,
                0, 0, 0, 1;

        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Tmap_cam = Tmap_aptag * Taptag_cam;
        Eigen::Vector4d point_wrt_map = Tmap_cam * point_wrt_camera_optical;
         std::cout << Tmap_cam << "   Tmap_cam   " << std::endl;
        std::cout << point_wrt_map << "   point_wrt_map   " << std::endl;
        }
    return 0;
}
//}