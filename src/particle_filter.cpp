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


#define EPSILON 1e-4


void ParticleFilter::init(std::pair<double, double> x_bound, std::pair<double, double> y_bound,
                          std::pair<double, double> z_bound,
                          std::pair<double, double> theta_bound) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    std::default_random_engine gen;
    std::uniform_real_distribution<double> xNoise(x_bound.first, x_bound.second);
    std::uniform_real_distribution<double> yNoise(y_bound.first, y_bound.second);
    std::uniform_real_distribution<double> zNoise(z_bound.first, z_bound.second);
    std::uniform_real_distribution<double> yawNoise(theta_bound.first, theta_bound.second);

    particles.clear();
    weights.clear();
    for (int i = 0; i < num_particles; ++i) {
        Particle p = {i, xNoise(gen), yNoise(gen),zNoise(gen),  yawNoise(gen), 1.0};
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

void enforce_non_collision(const std::vector<Particle> &part);

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
                                   std::vector<LandmarkObs> observations,  const Eigen::Matrix3d intrinsicParams,  const Eigen::Matrix4d extrinsicParams) {
    // Update the weights of each particle using a multi-variate Gaussian distribution. You can read

    double sigma_x = std_landmark[0];
    double sigma_y = std_landmark[1];
    double weights_sum = 0;

    // loop through each of the particle to update
    for(int i = 0; i < num_particles; ++i){
        Particle *p = &particles[i];
        double weight = 1.0;

        // project 3d point to pixels for now the size of observations will be one cause we are only tracking one joint which is the right shoulder
        // later we can add a kalman filter and use all the joints to predict the location of the person
//        for(int j=0; j<observations.size(); ++j) {
//            LandmarkObs current_obs = observations[j]; // in pixels in the image plane
//        }
        LandmarkObs current_obs = observations[0]; // TODO be changed when more observations are added
        Eigen::Vector4d particle;
        particle << p->x, p->y, p->z, 1;
        // projected particles to image plane

        Eigen::Vector2d predicted_particles =  projectParticlesto2D(particle, intrinsicParams,extrinsicParams);

        // update weights using Multivariate Gaussian Distribution
        // equation given in Transformations and Associations Quiz
        double gaussian = ((predicted_particles[0] - current_obs.x) * (predicted_particles[0] - current_obs.x) / (2 * sigma_x * sigma_x)) +
                              ((predicted_particles[1] - current_obs.y) * (predicted_particles[1] - current_obs.y) / (2 * sigma_x * sigma_y));
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


Eigen::Vector2d projectParticlesto2D(const Eigen::Vector4d& particle, const Eigen::Matrix3d& intrinsicParams, const Eigen::Matrix4d& extrinsicParams)
{
    Eigen::MatrixXd modifiedIntrinsicParams(3, 4);
    modifiedIntrinsicParams << intrinsicParams(0, 0), intrinsicParams(0, 1), intrinsicParams(0, 2), 0.0,
            intrinsicParams(1, 0), intrinsicParams(1, 1), intrinsicParams(1, 2), 0.0,
            intrinsicParams(2, 0), intrinsicParams(2, 1), intrinsicParams(2, 2), 0.0;

// Define the composite transformation matrix
    Eigen::Matrix<double, 4, 4> compositeMatrix = modifiedIntrinsicParams.matrix() * extrinsicParams;


    // Multiply the homogeneous 3D point with the matrix to get the projected 2D point
    Eigen::Vector4d projected_homog_point = compositeMatrix * particle;

    // Store the resulting 2D point in an Eigen::Vector2d object
    Eigen::Vector2d projectedPoint(projected_homog_point(0) / projected_homog_point(3), projected_homog_point(1) / projected_homog_point(3));

    return projectedPoint;
}



//void enforce_non_collision(const std::vector<Particle> &part){
//    auto network = trainable_model->get_network();
//    float *params = trainable_model->params();
//    for (int i = 0; i < 20; i++) {
//    predict(stream_ptr, network, params, features_gpu, pred_targets_gpu);
//    auto pred_targets = pred_targets_gpu.toCPU();
//    publish_pointcloud(features, pred_targets, node, pub_);
//
//    }
//
//    cudaStreamDestroy(*stream_ptr);
//}
//
//void predict(cudaStream_t const *stream_ptr, tcnn::cpp::Module *network,
//             float *params, const GPUData &inputs, GPUData &output) {
//
//    auto stream_ptr = new cudaStream_t();
//    cudaStreamCreate(stream_ptr);
//
//    auto batch_size = output.size_ / network->n_output_dims();
//    assert(output.size_ == network->n_output_dims() * batch_size);
//
//    output.stride_ = 16;
//    tcnn::cpp::Context ctx = network->forward(*stream_ptr, batch_size, inputs.data_gpu_, output.data_gpu_, params,
//                                              false);
//
//}
//
///**
// * @brief Load precomputed model weights from an external txt file.
// *
// * @param filename
// * @return std::vector<float>
// */
//std::vector<float> load_weights(const std::string &filename)
//{
//    std::cout << "Loading weights" << std::endl;
//    std::vector<float> weights_buffer;
//    std::ifstream in(filename);
//
//    for (std::string f; getline(in, f, '\n');)
//        weights_buffer.push_back(std::stof(f));
//
//    // for (auto f : weights_buffer)
//    // 	std::cout << f << ", ";
//    // std::cout << std::endl;
//    return weights_buffer;
//}
////// 0. Load precomputed network weights
//
////vector<float> weights = load_weights("../nerfdata/tinylego/weights.txt");
////trainer as in network
////trainer->set_params_full_precision(weights.data(), weights.size());
//// Note: These are column-major
////GPUMatrix<float> inference_batch(pts_vec.data(), n_input_dims, n_coords);
////GPUMatrix<float> prediction(n_output_dims, n_coords);
////or
////for (int i = 0; i < 20; i++) {
////predict(stream_ptr, network, params, features_gpu, pred_targets_gpu);
////auto pred_targets = pred_targets_gpu.toCPU();
////publish_pointcloud(features, pred_targets, node, pub_);
////
////}