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
                                  std::vector<bool> doors_status) {
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
    std::string ParamFilename = "network_params.json";
    std::string NetworkFilename = "network_config.json";
    ParticleFilter::enforce_non_collision(particles_before, ParamFilename,NetworkFilename, doors_status);

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
                                   std::vector<Observation> observations,
                                   Eigen::Matrix<double, 4, 4, Eigen::RowMajor> extrinsicParams) {
    // Update the weights of each particle using a multi-variate Gaussian distribution. You can read

    double sigma_x = std_landmark[0];
    double sigma_y = std_landmark[1];
    double sigma_z = std_landmark[2];
    double weights_sum = 0;

    Observation current_obs = observations[0]; // TODO be changed when more observations are added
    Eigen::Vector4d homogeneousPoint;
    homogeneousPoint << current_obs.x, current_obs.y, current_obs.z, 1.0;
    Eigen::Vector4d TransformedPoint;
    TransformedPoint << extrinsicParams(0,0) * homogeneousPoint[0] + extrinsicParams(0,1) * homogeneousPoint[1] + extrinsicParams(0,2) * homogeneousPoint[2] + extrinsicParams(0,3) * homogeneousPoint[3],
            extrinsicParams(1,0) * homogeneousPoint[0] + extrinsicParams(1,1) * homogeneousPoint[1] + extrinsicParams(1,2) * homogeneousPoint[2] + extrinsicParams(1,3) * homogeneousPoint[3],
            extrinsicParams(2,0) * homogeneousPoint[0] + extrinsicParams(2,1) * homogeneousPoint[1] + extrinsicParams(2,2) * homogeneousPoint[2] + extrinsicParams(2,3) * homogeneousPoint[3],
            extrinsicParams(3,0) * homogeneousPoint[0] + extrinsicParams(3,1) * homogeneousPoint[1] + extrinsicParams(3,2) * homogeneousPoint[2] + extrinsicParams(3,3) * homogeneousPoint[3];
    std::cout << " Observation ::: x " << TransformedPoint[0] << " y " << TransformedPoint[1] << " z " << TransformedPoint[2] << std::endl;


    // loop through each of the particle to update
    for (int i = 0; i < num_particles; ++i) {
        Particle *p = &particles[i];
        double weight = 1.0;

        // update weights using Multivariate Gaussian Distribution
        // equation given in Transformations and Associations Quiz
        double gaussian = ((p->x - TransformedPoint[0]) * (p->x - TransformedPoint[0]) /
                           (2 * sigma_x * sigma_x)) +
                          ((p->y - TransformedPoint[1]) * (p->y - TransformedPoint[1]) /
                           (2 * sigma_y * sigma_y)) + ((p->z - TransformedPoint[2]) * (p->z - TransformedPoint[2]) /
                                                       (2 * sigma_z * sigma_z));
        double gaussian_factor = 1 / (2 * M_PI * sigma_x * sigma_y * sigma_z);
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


void ParticleFilter::enforce_non_collision(const std::vector<Particle> &old_particles, std::string ParamFilename, std::string NetworkFilename,
                                           std::vector<bool> doors_status) {

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
    std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("particle_filter");
    auto json_file_path = pkg_dir / "config" / ParamFilename;
    std::ifstream inputFile(json_file_path.string());
    if (inputFile.is_open()) {
        inputFile >> jsonArray;
        inputFile.close();
    } else {
        std::cerr << "Unable to open output.json for reading" << std::endl;
    }
    std::vector<float> floatVector = jsonArray.get<std::vector<float>>();

    auto net_file_path = pkg_dir / "config" / NetworkFilename;
    std::string network_config_path = net_file_path.string();

    pred_targets = check_collision_inf(features_inf, targets_inf, 6, 3, network_config_path, floatVector);

    int num_targets = 6;
    for (int i = 0; i < pred_targets.size() / num_targets; i++) {
//        int size_ = pred_targets.size();

        assert(i * num_targets + 3 < pred_targets.size());

        if (pred_targets[i * num_targets + 1] > .5) {
            // non-empty
            if (pred_targets[i * num_targets + 2] > 0.5) {
                // collision door 1
                if (doors_status[0]) {
                    // door 1 closed keep old particles
                    particles[i] = old_particles[i];
                    particles[i].weight = 0.0;
                }
            } else if (pred_targets[i * num_targets + 3] > 0.5) {
                // collision door 2
                if (doors_status[1]) {
                    // door 2 closed keep old particles
                    particles[i] = old_particles[i];
                    particles[i].weight = 0.0;
                }
            } else if (pred_targets[i * num_targets + 4] > 0.5) {
                // collision door 3
                if (doors_status[2]) {
                    // door 1 closed keep old particles
                    particles[i] = old_particles[i];
                    particles[i].weight = 0.0;
                }
            } else if (pred_targets[i * num_targets + 5] > 0.5) {
                // collision door 4
                if (doors_status[3]) {
                    // door 1 closed keep old particles
                    particles[i] = old_particles[i];
                    particles[i].weight = 0.0;
                }
            } else {
                // not empty and not doors
                particles[i] = old_particles[i];
                particles[i].weight = 0.0;
            }
        }
    }
}

