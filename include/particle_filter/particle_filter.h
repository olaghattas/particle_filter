//
// Created by ola on 6/14/23.
//

#ifndef SMART_HOME_PARTICLE_FILTER_H
#define SMART_HOME_PARTICLE_FILTER_H

//#include <sstream>
//#include <fstream>
#include <math.h>
#include <vector>
//

struct Particle {

    int id;
    double x;
    double y;
    double z;
    double theta;
    double weight;
};

/*
     * Computes the Euclidean distance between two 2D points.
     * @param (x1,y1) x and y coordinates of first point
     * @param (x2,y2) x and y coordinates of second point
     * @output Euclidean distance between two 2D points
     */
inline double dist(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

class ParticleFilter{
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
    ParticleFilter(int num): num_particles(num), is_initialized(false) {}

    // Destructor
    ~ParticleFilter() {}

    void init(std::pair<double, double>  x, std::pair<double, double>  y, std::pair<double, double>  z, std::pair<double, double>  theta);
    void motion_model(double delta_t, std::array<double,4> std_pos, double velocity, double yaw_rate);
    void updateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations );
    void resample();
    void publish_particles(const std::vector<Particle> &particles);

    /**
	 * initialized Returns whether particle filter is initialized yet or not.
	 */
    const bool initialized() const {
        return is_initialized;
    }
    void enforce_non_collision(const std::vector<Particle> &part);


};


#endif //SMART_HOME_PARTICLE_FILTER_H
