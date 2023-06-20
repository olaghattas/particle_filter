//
// Created by ola on 6/14/23.
//
#include <random>
#include <algorithm>
#include <map>
#include <numeric>
#include <math.h>

#include "particle_filter/particle_filter.h"

#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <Eigen/Dense>

#define EPSILON 1e-4


void ParticleFilter::init(std::pair<double, double> x_bound, std::pair<double, double> y_bound,
                          std::pair<double, double> z_bound,
                          std::pair<double, double> theta_bound) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    std::default_random_engine gen;
    std::normal_distribution<double> xNoise(x_bound.first, x_bound.second);
    std::uniform_real_distribution<double> yNoise(y_bound.first, y_bound.second);
    std::uniform_real_distribution<double> zNoise(z_bound.first, z_bound.second);
    std::normal_distribution<double> yawNoise(theta_bound.first, theta_bound.second);

    particles.clear();
    weights.clear();
    for (int i = 0; i < num_particles; ++i) {
        Particle p = {i, xNoise(gen), yNoise(gen), yawNoise(gen), 1};
        particles.push_back(p);
        weights.push_back(1);
    }
    is_initialized = true;
}

void ParticleFilter::motion_model(double delta_t, std::array<double, 4> std_pos, double velocity, double yaw_rate) {
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

    enforce_non_collision(particles_before);
    //To do: restore collsion particles to their old values
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

// Function to perform the projection
std::vector<Eigen::Vector2d> projectParticlesto2D(const std::vector<Particle> &particles, const Eigen::Matrix3d &cameraMatrix)
{   std::vector<Eigen::Vector2d> projectedPoints;
    projectedPoints.reserve(particles.size());

    for (const auto& particle : particles) {
        Eigen::Vector2d point2D;
        point2D << (cameraMatrix(0, 0) * particle.x + cameraMatrix(0, 1) * particle.y + cameraMatrix(0, 2) * particle.z) / (cameraMatrix(2, 0) * particle.x + cameraMatrix(2, 1) * particle.y + cameraMatrix(2, 2) * particle.z),
                (cameraMatrix(1, 0) * particle.x + cameraMatrix(1, 1) * particle.y + cameraMatrix(1, 2) * particle.z) / (cameraMatrix(2, 0) * particle.x + cameraMatrix(2, 1) * particle.y + cameraMatrix(2, 2) * particle.z);
        projectedPoints.push_back(point2D);
    }
    return projectedPoints;
}