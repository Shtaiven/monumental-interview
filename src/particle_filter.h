#pragma once
#include <vector>
#include <random>
#include "types.h"

struct Particle {
    double x, y, theta, weight;
};

class ParticleFilter {
public:
    ParticleFilter(size_t num_particles = 100);

    void init(double x, double y, double theta, double std_pos[3]);
    void predict(double dt, double gyro_z, double acc_x, double acc_y, double std_motion[3]);
    void updateGPS(double gps_x, double gps_y, double std_gps[2]);
    Vec2 getPosition() const;
    double getOrientation() const;
    std::vector<Particle> getParticles() const;

private:
    std::vector<Particle> particles_;
    std::default_random_engine gen_;
};