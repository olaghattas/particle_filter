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

//To enforce collsion

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
//
//void predict(cudaStream_t const *stream_ptr, tcnn::cpp::Module *network,
//             float *params, const GPUData &inputs, GPUData &output) {
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
//
//std::vector<float> check_collision( std::vector<float> features_inf, std::vector<float> targets_inf , std::string directoryPath){
//    // load data
//    auto data = read_data_from_path(directoryPath);
//
//    std::vector<float> features(3 * data.size());
//    std::vector<float> targets(2 * data.size());
//    for (int i = 0; i < data.size(); i++) {
//        auto &datpoint = data[i];
//        targets[2 * i + 0] = datpoint.collision;
//        targets[2 * i + 1] = datpoint.door_collision;
//        features[3 * i + 0] = datpoint.x;
//        features[3 * i + 1] = datpoint.y;
//        features[3 * i + 2] = datpoint.z;
//
//    }
//
//    GPUData features_gpu{features, 3};
//    GPUData targets_gpu{targets, 2};
//    GPUData pred_targets_gpu((int) 16 * targets.size(), 1);
//
//    // load config
//    std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("collision");
//    auto json_file_path = pkg_dir / "config" / "config.json";
//    std::fstream file(json_file_path.string());
//    std::stringstream buffer;  // Create a stringstream to store the file contents
//    buffer << file.rdbuf();  // Read the file into the stringstream
//    std::string config_string = buffer.str(); // "{\"encoding\":{\"base_resolution\":16,\"log2_hashmap_size\":19,\"n_features_per_level\":2,\"n_levels\":16,\"otype\":\"HashGrid\",\"per_level_scale\":2.0},\"loss\":{\"otype\":\"L2\"},\"network\":{\"activation\":\"ReLU\",\"n_hidden_layers\":2,\"n_neurons\":64,\"otype\":\"FullyFusedMLP\",\"output_activation\":\"None\"},\"optimizer\":{\"learning_rate\":0.001,\"otype\":\"Adam\"}}";
//    nlohmann::json config = nlohmann::json::parse(config_string);
//
//    // load network and cuda
//    constexpr uint32_t n_input_dims = 3;
//    constexpr uint32_t n_output_dims = 1;
//    uint32_t batch_size = targets.size() / 2;
//
//    auto stream_ptr = new cudaStream_t();
//    cudaStreamCreate(stream_ptr);
//    auto trainable_model = tcnn::cpp::create_trainable_model(n_input_dims, n_output_dims, config);
//
//
//    for (int i = 0; i < 1000; ++i) {
//        int ind = features_gpu.sampleInd(batch_size);
//        float *training_batch_inputs = features_gpu.sample(ind);
//        float *training_batch_targets = targets_gpu.sample(ind);
//
//        auto ctx = trainable_model->training_step(*stream_ptr, batch_size, training_batch_inputs,
//                                                  training_batch_targets);
//        if (0 == i % 100) {
//            float loss = trainable_model->loss(*stream_ptr, ctx);
//            std::cout << "iteration=" << i << " loss=" << loss << std::endl;
//            auto network_config = config.value("network", nlohmann::json::object());
//            auto encoding_config = config.value("encoding", nlohmann::json::object());
//            auto network = tcnn::cpp::create_network_with_input_encoding(n_input_dims, n_output_dims, encoding_config,
//                                                                         network_config);
//            float *params = trainable_model->params();
//
//        }
//    }
//
//    // Inferencing
//
//    auto network = trainable_model->get_network();
//    float *params = trainable_model->params();
//
//    GPUData features_gpu_inf{features_inf, 3};
//    GPUData pred_targets_inf_gpu((int) 16 * targets_inf.size(), 1);
//
//
//    predict(stream_ptr, network, params, features_gpu_inf, pred_targets_inf_gpu);
//    auto pred_targets = pred_targets_gpu.toCPU();
//
//    cudaStreamDestroy(*stream_ptr);
//
//    return pred_targets;
//}
//
//std::vector<float> door_collision( std::vector<float> features_inf, std::vector<float> targets_inf , std::string directoryPath){
//    // load data
//    auto data = read_data_from_path(directoryPath);
//
//    std::vector<float> features(3 * data.size());
//    std::vector<float> targets(3 * data.size());
//    for (int i = 0; i < data.size(); i++) {
//        auto &datpoint = data[i];
//        targets[3 * i + 0] = datpoint.door1;
//        targets[3 * i + 1] = datpoint.door2;
//        targets[3 * i + 2] = datpoint.door3;
//        features[3 * i + 0] = datpoint.x;
//        features[3 * i + 1] = datpoint.y;
//        features[3 * i + 2] = datpoint.z;
//
//    }
//
//    GPUData features_gpu{features, 3};
//    GPUData targets_gpu{targets, 3};
//    GPUData pred_targets_gpu((int) 16 * targets.size(), 1);
//
//    // load config
//    std::filesystem::path pkg_dir = ament_index_cpp::get_package_share_directory("collision");
//    auto json_file_path = pkg_dir / "config" / "config.json";
//    std::fstream file(json_file_path.string());
//    std::stringstream buffer;  // Create a stringstream to store the file contents
//    buffer << file.rdbuf();  // Read the file into the stringstream
//    std::string config_string = buffer.str(); // "{\"encoding\":{\"base_resolution\":16,\"log2_hashmap_size\":19,\"n_features_per_level\":2,\"n_levels\":16,\"otype\":\"HashGrid\",\"per_level_scale\":2.0},\"loss\":{\"otype\":\"L2\"},\"network\":{\"activation\":\"ReLU\",\"n_hidden_layers\":2,\"n_neurons\":64,\"otype\":\"FullyFusedMLP\",\"output_activation\":\"None\"},\"optimizer\":{\"learning_rate\":0.001,\"otype\":\"Adam\"}}";
//    nlohmann::json config = nlohmann::json::parse(config_string);
//
//    // load network and cuda
//    constexpr uint32_t n_input_dims = 3;
//    constexpr uint32_t n_output_dims = 1;
//    uint32_t batch_size = targets.size() / 2;
//
//    auto stream_ptr = new cudaStream_t();
//    cudaStreamCreate(stream_ptr);
//    auto trainable_model = tcnn::cpp::create_trainable_model(n_input_dims, n_output_dims, config);
//
//
//    for (int i = 0; i < 1000; ++i) {
//        int ind = features_gpu.sampleInd(batch_size);
//        float *training_batch_inputs = features_gpu.sample(ind);
//        float *training_batch_targets = targets_gpu.sample(ind);
//
//        auto ctx = trainable_model->training_step(*stream_ptr, batch_size, training_batch_inputs,
//                                                  training_batch_targets);
//        if (0 == i % 100) {
//            float loss = trainable_model->loss(*stream_ptr, ctx);
//            std::cout << "iteration=" << i << " loss=" << loss << std::endl;
//            auto network_config = config.value("network", nlohmann::json::object());
//            auto encoding_config = config.value("encoding", nlohmann::json::object());
//            auto network = tcnn::cpp::create_network_with_input_encoding(n_input_dims, n_output_dims, encoding_config,
//                                                                         network_config);
//            float *params = trainable_model->params();
//
//        }
//    }
//
//    // Inferencing
//
//    auto network = trainable_model->get_network();
//    float *params = trainable_model->params();
//
//    GPUData features_gpu_inf{features_inf, 3};
//    GPUData pred_targets_inf_gpu((int) 16 * targets_inf.size(), 1);
//
//
//    predict(stream_ptr, network, params, features_gpu_inf, pred_targets_inf_gpu);
//    auto pred_targets = pred_targets_gpu.toCPU();
//
//    cudaStreamDestroy(*stream_ptr);
//
//    return pred_targets;
//}
//


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
    // to be passed in through arguments
    bool door_open = true;
    std::string directoryPath = "/home/ola/Desktop/unity_points/";

    ParticleFilter::enforce_non_collision(particles_before, directoryPath,  door_open);

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
                                   std::vector<LandmarkObs> observations,  const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> intrinsicParams,  const Eigen::Matrix4d extrinsicParams) {
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


Eigen::Vector2d ParticleFilter::projectParticlesto2D(const Eigen::Vector4d& particle, const Eigen::Matrix3d& intrinsicParams, const Eigen::Matrix4d& extrinsicParams)
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

//void ParticleFilter::enforce_non_collision(const std::vector<Particle> &old_particles, std::string directoryPath, bool door_close){

//}

// the function runs training before inferecinG it will then be separated
void ParticleFilter::enforce_non_collision(const std::vector<Particle> &old_particles, std::string directoryPath, bool door_close){

    std::vector<float> features_inf(3 * num_particles);
    std::vector<float> targets_inf(2 * num_particles);

    for (int i = 0; i < num_particles; ++i) {
        auto &datapoint = particles[i];
        // particles are double while nn takes float
        features_inf[3 * i + 0] = static_cast<float>(datapoint.x);
        features_inf[3 * i + 1] = static_cast<float>(datapoint.y);
        features_inf[3 * i + 2] = static_cast<float>(datapoint.z);
    }

    std::vector<float> pred_targets;
    pred_targets = check_collision( features_inf, targets_inf, directoryPath);

    for (int i = 0; i < pred_targets.size() / 2; i++) {
        float point[3] = {features_inf[3 * i + 0], features_inf[3 * i + 1], features_inf[3 * i + 2]};

        // number of pred_targets = num_particles * 2, each particle has two outputs
        if (pred_targets[i * 2] > .5){
            // if collision is with a door
            if (pred_targets[i * 2 + 1] > 0.5)
            {
                // if door is closed keep the old points
                if (door_close){
                particles[i] = old_particles[i * 2 + 1];
                }
            }
            // if collision and the collision is NOT with a door
            else {
                //use the old values
                particles[i] = old_particles[i * 2 + 1];
            }
        }
    }
}



