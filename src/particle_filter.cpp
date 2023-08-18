//
// Created by ola on 6/14/23.
//
#include <random>
#include <algorithm>
#include <map>
#include <numeric>
#include "particle_filter/particle_filter.h"
#include <functional>
#include <memory>
#include <string>
#include <cmath>

// needed for the projection
#include <iostream>






#define EPSILON 1e-4

//To enforce collision

void ParticleFilter::init(std::pair<double, double> x_bound, std::pair<double, double> y_bound,
                          std::pair<double, double> z_bound,
                          std::pair<double, double> theta_bound) {

    // Add random Gaussian noise to each particle.
    std::default_random_engine gen;
    std::uniform_real_distribution<double> xNoise(x_bound.first, x_bound.second);
    std::uniform_real_distribution<double> yNoise(y_bound.first, y_bound.second);
    std::uniform_real_distribution<double> zNoise(z_bound.first, z_bound.second);
    std::uniform_real_distribution<double> yawNoise(theta_bound.first, theta_bound.second);

    particles.clear();
    weights.clear();
    for (int i = 0; i < num_particles; ++i) {
        Particle p = {i, xNoise(gen), yNoise(gen), zNoise(gen), yawNoise(gen), 1.0};
        particles.push_back(p);
        weights.push_back(1);
    }
    is_initialized = true;
}

void ParticleFilter::motion_model(double delta_t, std::array<double, 4> std_pos, double velocity, double yaw_rate,
                                   tcnn::cpp::Module *network, std::vector<bool> doors_status) {
    std::default_random_engine gen;
    std::normal_distribution<double> xNoise(0, std_pos[0]);
    std::normal_distribution<double> yNoise(0, std_pos[1]);
    std::normal_distribution<double> zNoise(0, std_pos[2]);
    std::normal_distribution<double> yawNoise(0, std_pos[3]);
    auto particles_before = particles;

    for (auto &p: particles) {
        double yaw = p.theta;

        double delta_x = 0;
        double delta_y = 0;
        double delta_z = 0;
        double delta_yaw = 0;

        if (fabs(yaw_rate) < EPSILON) {
            delta_x = velocity * delta_t * cos(yaw);
            delta_y = velocity * delta_t * sin(yaw);

        } else {
            double c = velocity / yaw_rate;
            delta_x = c * (sin(yaw + yaw_rate * delta_t) - sin(yaw));
            delta_y = c * (cos(yaw) - cos(yaw + yaw_rate * delta_t));
            delta_yaw = yaw_rate * delta_t;

        }
        //Add control noise
        delta_x += xNoise(gen) * delta_t;
        delta_y += yNoise(gen) * delta_t;
        delta_z += zNoise(gen) * delta_t;
        delta_yaw += yawNoise(gen) * delta_t;


        p.x += delta_x;
        p.y += delta_y;
        p.z += delta_z;
        p.theta += delta_yaw;
    }

    // to be passed in through arguments
    bool door_open = true;
    std::string directoryPath = "/home/ola/Desktop/unity_points/";

    ParticleFilter::enforce_non_collision(particles_before, directoryPath, doors_status, network);

}

void ParticleFilter::resample() {

    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    std::default_random_engine gen;
    std::discrete_distribution<int> d(weights.begin(), weights.end());
    std::vector<Particle> resampled_particles(particles.size());
    for (int i = 0; i < num_particles; ++i) {
        int idx = d(gen);
        resampled_particles[i] = particles[idx];
    }
    particles = resampled_particles;
}

void ParticleFilter::updateWeights(double std_landmark[],
                                   std::vector<LandmarkObs> observations,
                                   const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> intrinsicParams,
                                   Eigen::Matrix4d extrinsicParams) {
    // Update the weights of each particle using a multi-variate Gaussian distribution. You can read

    double sigma_x = std_landmark[0];
    double sigma_y = std_landmark[1];
    double weights_sum = 0;

    // loop through each of the particle to update
    for (int i = 0; i < num_particles; ++i) {
        Particle *p = &particles[i];
        double weight = 1.0;

        // project 3d point to pixels for now the size of observations will be one cause we are only tracking one joint which is the right shoulder
        // later we can add a kalman filter and use all the joints to predict the location of the person
//        for(int j=0; j<observations.size(); ++j) {
//            LandmarkObs current_obs = observations[j]; // in pixels in the image plane
//        }
        LandmarkObs current_obs = observations[0]; // TODO be changed when more observations are added

        /// has to be in a vector for rojecton cv2

        // projected particles to image plane

        std::vector<cv::Point2d> predicted_particles = projectParticlesto2D(*p, intrinsicParams, extrinsicParams);

        // update weights using Multivariate Gaussian Distribution
        // equation given in Transformations and Associations Quiz
        double gaussian = ((predicted_particles[0].x - current_obs.x) * (predicted_particles[0].x - current_obs.x) /
                           (2 * sigma_x * sigma_x)) +
                          ((predicted_particles[0].y - current_obs.y) * (predicted_particles[0].y - current_obs.y) /
                           (2 * sigma_x * sigma_y));
        double gaussian_factor = 1 / (2 * M_PI * sigma_x * sigma_y);
        gaussian = exp(-gaussian);
        gaussian = gaussian * gaussian_factor;

        weight *= gaussian;
        weights_sum += weight;
        p->weight = weight;
    }


    // normalize weights to bring them in (0, 1]
    for (int i = 0; i < num_particles; i++) {
        Particle *p = &particles[i];
        p->weight /= weights_sum;
        weights[i] = p->weight;
    }
}

// Convert Eigen matrix to OpenCV matrix
cv::Mat eigenToCv(const Eigen::MatrixXd& eigenMat) {
    cv::Mat cvMat(eigenMat.rows(), eigenMat.cols(), CV_64F);
    for (int i = 0; i < eigenMat.rows(); ++i) {
        for (int j = 0; j < eigenMat.cols(); ++j) {
            cvMat.at<double>(i, j) = eigenMat(i, j);
        }
    }
    return cvMat;
}

std::vector<cv::Point2d>
ParticleFilter::projectParticlesto2D(const Particle particle_, const Eigen::Matrix3d &intrinsicMat,
                                     const Eigen::Matrix4d &extrinsicParams) {

    std::vector<cv::Point3d> objectPoints;
    objectPoints.push_back(cv::Point3d(particle_.x, particle_.y, particle_.z));

    std::vector<cv::Point2d> imagePoints;

    // Define camera extrinsics using Eigen
    // Extract the rotation part (3x3 submatrix) from the extrinsicParams
    Eigen::Matrix3d R = extrinsicParams.block<3, 3>(0, 0);

    // Convert the rotation matrix to Rodrigues vector format
//    cv::Mat_<double> rotationMat(3, 3);
    auto rotationMat = eigenToCv(R);
    cv::Mat rvec(3, 1, cv::DataType<double>::type);
    cv::Rodrigues(rotationMat, rvec);


    // Extract the translation part (last column) from the extrinsicParams
    cv::Mat tvec(3, 1, cv::DataType<double>::type);
    tvec.at<double>(0, 0) = extrinsicParams(0, 3);
    tvec.at<double>(1, 0) = extrinsicParams(1, 3);
    tvec.at<double>(2, 0) = extrinsicParams(2, 3);

    // Convert Eigen intrinsic matrix to OpenCV format (3x3)
    cv::Mat cvIntrinsicMat = (cv::Mat_<double>(3, 3) << intrinsicMat(0, 0), intrinsicMat(0, 1), intrinsicMat(0, 2),
            intrinsicMat(1, 0), intrinsicMat(1, 1), intrinsicMat(1, 2),
            intrinsicMat(2, 0), intrinsicMat(2, 1), intrinsicMat(2, 2));

    cv::projectPoints(objectPoints, rvec, tvec, cvIntrinsicMat, cv::Mat(), imagePoints);

    return imagePoints;
}

// TODO; Separate training and inferencing
void ParticleFilter::enforce_non_collision(const std::vector<Particle> &old_particles, std::string directoryPath,
                                           std::vector<bool> doors_status, tcnn::cpp::Module *network) {

    std::vector<float> features_inf(3 * num_particles);
    std::vector<float> targets_inf(6 * num_particles);

    for (int i = 0; i < num_particles; ++i) {
        auto &datapoint = particles[i];
        // particles are double while nn takes float
        features_inf[3 * i + 0] = static_cast<float>(datapoint.x);
        features_inf[3 * i + 1] = static_cast<float>(datapoint.y);
        features_inf[3 * i + 2] = static_cast<float>(datapoint.z);
    }

    std::vector<float> pred_targets;

    nlohmann::json jsonArray;
    /// path to params file
    std::string path = "/home/olagh/particle_filter/src/neural_collision/collision_lib/config/output.json";
    std::ifstream inputFile(directoryPath);
    if (inputFile.is_open()) {
        inputFile >> jsonArray;
        inputFile.close();
    } else {
        std::cerr << "Unable to open output.json for reading" << std::endl;
    }
    std::vector<float> floatVector = jsonArray.get<std::vector<float>>();

    pred_targets = check_collision_inf(features_inf, targets_inf, 6, 3, network, floatVector);

    int num_targets = 6;
    for (int i = 0; i < pred_targets.size() / num_targets; i++) {
        int size_ = pred_targets.size();

        assert(i * num_targets + 3 < pred_targets.size());

        if (pred_targets[i * num_targets + 1] > .5) {
            // non-empty
            if (pred_targets[i * num_targets + 2] > 0.5) {
                // collision door 1
                if (doors_status[0]) {
                    // door 1 closed keep old particles
                    particles[i] = old_particles[i];
                }
            } else if (pred_targets[i * num_targets + 3] > 0.5) {
                // collision door 2
                if (doors_status[1]) {
                    // door 2 closed keep old particles
                    particles[i] = old_particles[i];
                }
            } else if (pred_targets[i * num_targets + 4] > 0.5) {
                // collision door 3
                if (doors_status[2]) {
                    // door 1 closed keep old particles
                    particles[i] = old_particles[i];
                }
            } else if (pred_targets[i * num_targets + 5] > 0.5) {
                // collision door 4
                if (doors_status[3]) {
                    // door 1 closed keep old particles
                    particles[i] = old_particles[i];
                }
            } else {
                // not empty and not doors
                particles[i] = old_particles[i];
            }
        }
    }
}

//
//Eigen::Vector2d ParticleFilter::projectParticlesto2D(const Eigen::Vector4d& particle, const Eigen::Matrix3d& intrinsicParams, const Eigen::Matrix4d& extrinsicParams)
//{
//    Eigen::MatrixXd modifiedIntrinsicParams(3, 4);
//    modifiedIntrinsicParams << intrinsicParams(0, 0), intrinsicParams(0, 1), intrinsicParams(0, 2), 0.0,
//            intrinsicParams(1, 0), intrinsicParams(1, 1), intrinsicParams(1, 2), 0.0,
//            intrinsicParams(2, 0), intrinsicParams(2, 1), intrinsicParams(2, 2), 0.0;
//
//// Define the composite transformation matrix
//    Eigen::Matrix<double, 4, 4> compositeMatrix = modifiedIntrinsicParams.matrix() * extrinsicParams;
//
//
//    // Multiply the homogeneous 3D point with the matrix to get the projected 2D point
//    Eigen::Vector4d projected_homog_point = compositeMatrix * particle;
//
//    // Store the resulting 2D point in an Eigen::Vector2d object
//    Eigen::Vector2d projectedPoint(projected_homog_point(0) / projected_homog_point(2), projected_homog_point(1) / projected_homog_point(2));
//
//    return projectedPoint;
//}

