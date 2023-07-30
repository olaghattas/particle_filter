//
// Created by ola on 6/14/23.
//

#ifndef SMART_HOME_PARTICLE_FILTER_H
#define SMART_HOME_PARTICLE_FILTER_H

#include <math.h>
#include <vector>
#include <array>
//
#include <cuda_fp16.h>
#include "tiny-cuda-nn/cpp_api.h"
#include <collision_lib/collision_lib.hpp>
//#include <get_collision.cpp>
//#include <collision/datapoint.hpp>
//#include <collision/get_collision.hpp>
#include <filesystem>
#include <Eigen/Dense>

struct Particle {
    int id;
    double x;
    double y;
    double z;
    double theta;
    double weight;
};

struct LandmarkObs {

    std::string name;        // Id of matching landmark. landmark in our case is the joint we are sarting with on ebut later will include all joints
    int x;      // x position of landmark (joint) in pixels
    int y;      // y position of landmark (joint) in pixels
}; // going to be 1x2 for now (left shoulder joint)

/*
     * Computes the Euclidean distance between two 2D points.
     * @param (x1,y1) x and y coordinates of first point
     * @param (x2,y2) x and y coordinates of second point
     * @output Euclidean distance between two 2D points
     */
inline double dist(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}


class ParticleFilter {
private:
    // Number of particles to draw
    int num_particles;

    // Flag, if filter is initialized
    bool is_initialized;

    // Vector of weights of all particles
    std::vector<double> weights;

    // Set of current particles


public:
    std::vector<Particle> particles;

    // Constructor
    ParticleFilter(int num) : num_particles(num), is_initialized(false) {}

    // Destructor
    ~ParticleFilter() {}

    void init(std::pair<double, double> x, std::pair<double, double> y, std::pair<double, double> z,
              std::pair<double, double> theta);

    void motion_model(double delta_t, std::array<double, 4> std_pos, double velocity, double yaw_rate);

    void updateWeights(double std_landmark[],
                       std::vector<LandmarkObs> observations, const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> intrinsicParams,
                       Eigen::Matrix4d extrinsicParams);

    void resample();

    /**
	 * initialized Returns whether particle filter is initialized yet or not.
	 */
    bool initialized() const {
        return is_initialized;
    }

//    void enforce_non_collision(const std::vector<Particle> &part);

    void enforce_non_collision(const std::vector<Particle> &old_particles, std::string directoryPath, bool door_close);

    Eigen::Vector2d projectParticlesto2D(const Eigen::Vector4d &particle, const Eigen::Matrix3d &intrinsicParams,
                                         const Eigen::Matrix4d &extrinsicParams);


};


#endif //SMART_HOME_PARTICLE_FILTER_H
