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
#include <collision/datapoint.hpp>
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

//struct GPUData {
//    GPUData(std::vector<float> data, int dim) {
//        size_ = data.size();
//        stride_ = 1;
//        dim_ = dim;
//        cudaMalloc(&data_gpu_, size_ * sizeof(float));
//        cudaMemcpy(data_gpu_, data.data(), size_ * sizeof(float), cudaMemcpyHostToDevice);
//
//        std::srand(static_cast<unsigned>(std::time(nullptr)));
//    }
//
//    GPUData(int size, int dim) {
//        size_ = size;
//        stride_ = 1;
//        dim_ = dim;
//        cudaMalloc(&data_gpu_, size_ * sizeof(float));
//    }
//
//    ~GPUData() {
//        cudaFree(data_gpu_);
//    }
//
//    int sampleInd(int num_elements) {
//        assert(size_ / dim_ - num_elements >= 0);
//        int offset = (std::rand() % (1 + size_ / dim_ - num_elements));
//        return offset;
//    }
//
//    float *sample(int offset) {
//        return (float *) (data_gpu_ + offset * dim_);
//    }
//
//    std::vector<float> toCPU() {
//        std::vector<float> out(size_ / stride_);
//        if (stride_ == 1) {
//            cudaMemcpy(out.data(), data_gpu_, size_ * sizeof(float), cudaMemcpyDeviceToHost);
//        } else {
//            std::vector<float> buf(size_);
//            cudaMemcpy(buf.data(), data_gpu_, size_ * sizeof(float), cudaMemcpyDeviceToHost);
//            for (int i = 0; i < size_ / stride_; i++) {
//                out[i] = buf[stride_ * i];
//            }
//        }
//
//        return out;
//    }
//
//    float *data_gpu_;
//    int size_;
//    int dim_;
//    int stride_;
//};

struct LandmarkObs {

    std::string name;        // Id of matching landmark. landmark in our case is the joint we are sarting with on ebut later will include all joints
    double x;      // x position of landmark (joint) in camera coordinates
    double y;      // y position of landmark (joint) in camera coordinates
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
    std::vector<Particle> particles;

public:
    // Constructor
    ParticleFilter(int num) : num_particles(num), is_initialized(false) {}

    // Destructor
    ~ParticleFilter() {}

    void init(std::pair<double, double> x, std::pair<double, double> y, std::pair<double, double> z,
              std::pair<double, double> theta);

    void motion_model(double delta_t, std::array<double, 4> std_pos, double velocity, double yaw_rate);

    void updateWeights(double std_landmark[],
                       std::vector<LandmarkObs> observations, const Eigen::Matrix3d intrinsicParams,
                       const Eigen::Matrix4d extrinsicParams);

    void resample();

    void publish_particles(const std::vector<Particle> &particles);

    /**
	 * initialized Returns whether particle filter is initialized yet or not.
	 */
    const bool initialized() const {
        return is_initialized;
    }

    void enforce_non_collision(const std::vector<Particle> &part);

    Eigen::Vector2d projectParticlesto2D(const Eigen::Vector4d &particle, const Eigen::Matrix3d &intrinsicParams,
                                         const Eigen::Matrix4d &extrinsicParams);

//    void predict(cudaStream_t const *stream_ptr, tcnn::cpp::Module *network,
//                 float *params, const GPUData &inputs, GPUData &output);


};


#endif //SMART_HOME_PARTICLE_FILTER_H
