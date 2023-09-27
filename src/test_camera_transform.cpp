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
    std::map<std::string, Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> cameraextrinsics;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cameraintrinsics;
    LandmarkObs observation; // Member variable to store the observation

//    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<detection_msgs::msg::PoseMsg>::SharedPtr pose_sub_lr;

//    std::vector<bool> door_status_;
    bool door_outdoor;
    bool door_livingroom;
    bool door_bedroom;
    bool door_bathroom;
    sensor_msgs::msg::Image::SharedPtr image_;


public:
    ParticleFilterNode() : Node("particle_filter") {

//        auto func = [this]() -> void { cam_extrinsics_from_tf(); };
//        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), func);

        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pose_sub_lr = create_subscription<detection_msgs::msg::PoseMsg>(
                "/coord_shoulder_joint_in_px_livingroom", 10,
                [this](const detection_msgs::msg::PoseMsg::SharedPtr msg) { PosePixCallback_livingroom(msg); });

//// TODO: change topic to jetson cameras
//        auto cameraInfoSub = create_subscription<se_nsor_msgs::msg::CameraInfo>(
//                "/camera_dining/color/camera_info", 1,
//                [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) { cameraInfoCallback(msg); });

        /// for validation particle filter
        camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
                "/camera_dining/color/image_raw", 10,
                [this](const sensor_msgs::msg::Image::SharedPtr msg) { cameraImgCallback(msg); });
    }

    LandmarkObs getObservation() const {
        return observation;
    }

    void PosePixCallback_livingroom(const detection_msgs::msg::PoseMsg::SharedPtr &msg) {

        std::cout << "msg &&&&&&&&&&&&&&&&&&&&  \n" << std::endl;

        observation.name = msg->name;
        observation.x = msg->pixel_coordinate_x;
        observation.y = msg->pixel_coordinate_y;

    }


    void publish_particles(std::vector<Particle> &particles) {       // Create a marker array message
        auto markerArrayMsg = std::make_shared<visualization_msgs::msg::MarkerArray>();
        // Populate the marker array with markers
        for (const auto &particle: particles) {
            // Create a marker message
            visualization_msgs::msg::Marker marker;

            // Set the marker properties
            marker.header.frame_id = "unity";
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

//    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr &msg) {
//        // Access camera matrix values
//        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> K(msg->k.data());
//        cameraintrinsics = K;
//    }

//    std::map<std::string, Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> cam_extrinsics_from_tf() {
//        /// Test
//
//        std::vector<std::string> cams{"rgb_camera_dining"};
//
//        // Loop over the keys of map_cam_aptag using a range-based for loop
//        for (const auto &cam: cams) {
//            // Get transformation matrix from camera to aptag
////            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> t_aptag_to_cam = transform_tf("tag_1", cam);
//
//
//            // Get transformation matrix from map to waptag
//            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> t_map_to_waptag = transform_tf("unity", cam);
//
//            // Get transformation matrix from map to aptag
////            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> t_map_to_cam = t_map_to_waptag * t_aptag_to_cam;
//
////            cameraextrinsics.insert(std::make_pair(cam, t_map_to_cam));
////            cameraextrinsics.insert(std::make_pair("name", t_map_to_cam));
//            cameraextrinsics.insert(std::make_pair("name", t_map_to_waptag));
//            return cameraextrinsics;
//
//        }
//    }


//    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> transform_tf(std::string toFrame, std::string fromFrame) {
//
//        try {
//            // get the geometry transform frames
//            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
//                    toFrame, fromFrame,
//                    tf2::TimePoint(), std::chrono::milliseconds(50));
//
//            geometry_msgs::msg::Transform transform_ = t.transform;
//
//            // turn geometry transform to 4x4 matrix
//            Eigen::Matrix<double, 4, 4, Eigen::RowMajor> transform = transform_geometry_to_matrix(transform_);
//            RCLCPP_INFO(this->get_logger(), "transform %s to %s", fromFrame.c_str(), toFrame.c_str());
//
//            return transform;
//
//        }
//        catch (const tf2::TransformException &ex) {
//            RCLCPP_INFO(
//                    this->get_logger(), "Could not transform %s to %s: %s",
//                    fromFrame.c_str(), toFrame.c_str(), ex.what());
////            return;
//        }
//    }

//    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> get_cam_intrinsic_matrix() {
//        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cameraintrinsics_;
//        cameraintrinsics_ << 415.69219771027923, 0.0, 320.0,
//                0.0, 415.69219771027923, 240.0,
//                0.0, 0.0, 1.0;
//        return cameraintrinsics_;
//    }


//    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> transform_geometry_to_matrix(geometry_msgs::msg::Transform transform) {
//        Eigen::Matrix<double, 4, 4, Eigen::RowMajor> extrinsicmatrix;
//        Eigen::Quaterniond quaternion(transform.rotation.w,
//                                      transform.rotation.x,
//                                      transform.rotation.y,
//                                      transform.rotation.z);
//        Eigen::Matrix3d rotationMatrix = quaternion.normalized().toRotationMatrix();
//        Eigen::Vector3d translationVector(transform.translation.x,
//                                          transform.translation.y,
//                                          transform.translation.z);
//
//        extrinsicmatrix.block<3, 3>(0, 0) = rotationMatrix;
//        extrinsicmatrix.block<3, 1>(0, 3) = translationVector;
//        extrinsicmatrix.row(3) << 0, 0, 0, 1;
//        return extrinsicmatrix;
//    }

    void cameraImgCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::cout << "raheeb  \n" << std::endl;
        if (msg) {
            // Convert the sensor_msgs::Image to a cv::Mat using cv_bridge
            image_ = msg;
//            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);

            // Access the cv::Mat containing the image data
//            img = cv_ptr->image;
//            std::cout << "Image Type: " << cv::typeToString(img.type()) << std::endl;
            // Print the contents of the image
//            for (int i = 0; i < img.rows; i++) {
//                for (int j = 0; j < img.cols; j++) {
//                    cv::Vec3b pixel = img.at<cv::Vec3b>(i, j);
//                    std::cout << "Pixel at (" << i << ", " << j << "): ";
//                    std::cout << "B = " << static_cast<int>(pixel[0]) << ", ";
//                    std::cout << "G = " << static_cast<int>(pixel[1]) << ", ";
//                    std::cout << "R = " << static_cast<int>(pixel[2]) << std::endl;
//                }
//            }
//            cv::imshow("Window Name", img);
//            cv::waitKey(0);
//            cv::destroyAllWindows();

//            std::cout << image << " image   \n" << std::endl;



            // Now you can work with the 'image' cv::Mat
            // For example, you can display it or perform image processing operations
        } else {

            RCLCPP_ERROR(get_logger(), "no image ");
        }
    }

    sensor_msgs::msg::Image::SharedPtr getImage() {
//        std::cout << "raheeb3  \n" << std::endl;
//        std::cout << image << " image   \n" << std::endl;
//        if (!image.empty()){
//                    std::cout << "raheeb4 \n" << std::endl;
////        std::cout << image << " image   \n" << std::endl;
//       }
        return image_;
    }
};


/// to create custom data for testing projection of particles

struct Point3D {
    double x, y, z;

    Point3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
};

std::vector<Point3D> samplePointsOnLine(const Point3D &start, const Point3D &end, int numPoints) {
    std::vector<Point3D> points;

    for (int i = 0; i < numPoints; ++i) {
        double t = static_cast<double>(i) / (numPoints - 1); // Interpolation parameter [0, 1]

        // Interpolate between start and end points
        double interpolatedX = start.x + t * (end.x - start.x);
        double interpolatedY = start.y + t * (end.y - start.y);
        double interpolatedZ = start.z + t * (end.z - start.z);

        points.emplace_back(interpolatedX, interpolatedY, interpolatedZ);
    }

    return points;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);


    auto node = std::make_shared<ParticleFilterNode>();
//    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
//    cv::Mat image;
    sensor_msgs::msg::Image::SharedPtr msg;
    cv::Mat image;
    // Access the cv::Mat containing the image data



    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> camera_extrinsics;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_intrinsics;

    camera_extrinsics << 0.000, -1.000, 0.000, -0.590,
            1.000, 0.000, 0.000, -6.930,
            0.000, 0.000, 1.000, -0.386,
            0.000, 0.000, 0.000, 1.000;

    camera_intrinsics << 415.69219771027923, 0.0, 320.0,
            0.0, 415.69219771027923, 240.0,
            0.0, 0.0, 1.0;


    // Todo map observation to camera intrinsic and extrinsics
    //    std::map<std::string, cv::Mat> cameraExtrinsics;
    //    cameraExtrinsics.insert(std::make_pair("dining", result_dining));

    while (rclcpp::ok()) {

        int num_particles = 128; // has to be multiple of 128

        double velocity = 0.1;
        double yaw_rate = 1.0;
        bool running = true;
        int count = 0;

        // test projection
        while (running) {

            std::vector<LandmarkObs> observations;

//            std::cout << "Image Type: " << cv::typeToString(image.type()) << std::endl;
//            std::cout << "Image Size (Rows x Cols): " << image.rows << " x " << image.cols << std::endl;


            // hardcoded for testing
            LandmarkObs obs_;
            obs_.name = "name";
            obs_.x = 300.0;
            obs_.y = 200.0;

            double sigma_landmark[2] = {100, 100};

            // observation will always be from the same camera
            std::string cam_name = obs_.name;
            observations.push_back(obs_);

            // limits for reference
//                std::tuple<int, int, int>  start = std::make_tuple(-5, -7, -1);
//                std::tuple<int, int, int>  end = std::make_tuple(5, 7, 1);
            std::tuple<int, int, int> start = std::make_tuple(-5, -7, -0.5);
            std::tuple<int, int, int> end = std::make_tuple(5, 7, -0.6);
            ParticleFilter particle_filter(num_particles);
            particle_filter.particles.clear();

//            for (int i = 0; i < num_particles; ++i) {
//                double t = static_cast<double>(i) / (num_particles - 1); // Interpolation parameter [0, 1]
//
//                // Interpolate between start and end points
//                double interpolatedX = std::get<0>(start) + t * (std::get<0>(end) - std::get<0>(start));
//                double interpolatedY = std::get<1>(start) + t * (std::get<1>(end) - std::get<1>(start));
//                double interpolatedZ = std::get<2>(start) + t * (std::get<2>(end) - std::get<2>(start));
//                Particle p = {i, interpolatedX, interpolatedY, interpolatedZ, 1.0, 1.0};
//                particle_filter.particles.push_back(p);
//                std::cout << "push back particlce  \n" << std::endl;
//            }

            Particle p = {0, -0.475, -3.486, 0.0528, 1.0, 1.0};
            particle_filter.particles.push_back(p);

            node->publish_particles(particle_filter.particles);
            for (auto par: particle_filter.particles) {
                particle_filter.projectParticlesto2D(par, camera_intrinsics, camera_extrinsics);
            }

            exec.spin_once();
        }
//        while (running) {
//            auto beg = std::chrono::high_resolution_clock::now();
//            msg = node->getImage();
//
//            std::cout  << "count" << count++ << std::endl;
//
//
//            if (msg) {
//                cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
//                image = cv_ptr->image;
//
////                cv::imshow("Window Name", image);
////                cv::waitKey(0);
////                cv::destroyAllWindows();
//
//                std::vector<LandmarkObs> observations;
//
//                std::cout << "Image Type: " << cv::typeToString(image.type()) << std::endl;
//                std::cout << "Image Size (Rows x Cols): " << image.rows << " x " << image.cols << std::endl;
//
//                LandmarkObs obs_ = node->getObservation();
//
//                double sigma_landmark[2] = {100, 100};
//
//                // observation will always be from the same camera
//                std::string cam_name = obs_.name;
//                observations.push_back(obs_);
//
//
//                // limits for reference
////                std::tuple<int, int, int>  start = std::make_tuple(-5, -7, -1);
////                std::tuple<int, int, int>  end = std::make_tuple(5, 7, 1);
//                std::tuple<int, int, int> start = std::make_tuple(-5, -7, -0.5);
//                std::tuple<int, int, int> end = std::make_tuple(5, 7, -0.6);
//                ParticleFilter particle_filter(num_particles);
//                particle_filter.particles.clear();
//
//                for (int i = 0; i < num_particles; ++i) {
//                    double t = static_cast<double>(i) / (num_particles - 1); // Interpolation parameter [0, 1]
//
//                    // Interpolate between start and end points
//                    double interpolatedX = std::get<0>(start) + t * (std::get<0>(end) - std::get<0>(start));
//                    double interpolatedY = std::get<1>(start) + t * (std::get<1>(end) - std::get<1>(start));
//                    double interpolatedZ = std::get<2>(start) + t * (std::get<2>(end) - std::get<2>(start));
//                    Particle p = {i, interpolatedX, interpolatedY, interpolatedZ, 1.0, 1.0};
//                    particle_filter.particles.push_back(p);
////                    std::cout << "push back particlce  \n" << std::endl;
//                }
//                node->publish_particles(particle_filter.particles);
//                particle_filter.updateWeights_mod_debugging(sigma_landmark,
//                                                            observations,camera_intrinsics, camera_extrinsics,
//                                                            image);
//                // Update the weights and resample
//
//            } else {
//                std::cout << "No image found yet \n" << std::endl;
//            }
//            exec.spin_once();
//        }
        rclcpp::shutdown();
        return 0;
    }

}