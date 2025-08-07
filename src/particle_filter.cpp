#include "particle_filter.h"

#include <algorithm>
#include <cmath>
#include <numeric>

ParticleFilter::ParticleFilter(size_t num_particles) {
    particles_.resize(num_particles);
}

void ParticleFilter::init(double x, double y, double theta, double std_pos[3]) {
    std::normal_distribution<double> dist_x(x, std_pos[0]);
    std::normal_distribution<double> dist_y(y, std_pos[1]);
    std::normal_distribution<double> dist_theta(theta, std_pos[2]);
    for (auto& p : particles_) {
        p.x = dist_x(gen_);
        p.y = dist_y(gen_);
        p.theta = dist_theta(gen_);
        p.weight = 1.0;
    }
}

void ParticleFilter::predict(double dt, double gyro_z, double acc_x,
                             double acc_y, double std_motion[3]) {
    std::normal_distribution<double> noise_x(0, std_motion[0]);
    std::normal_distribution<double> noise_y(0, std_motion[1]);
    std::normal_distribution<double> noise_theta(0, std_motion[2]);
    for (auto& p : particles_) {
        // Update theta first
        p.theta += gyro_z * dt + noise_theta(gen_);

        // Rotate acceleration to global frame
        double ax_global =
            std::cos(p.theta) * acc_x - std::sin(p.theta) * acc_y;
        double ay_global =
            std::sin(p.theta) * acc_x + std::cos(p.theta) * acc_y;

        // Simple integration (no velocity state)
        p.x += 0.5 * ax_global * dt * dt + noise_x(gen_);
        p.y += 0.5 * ay_global * dt * dt + noise_y(gen_);
    }
}

void ParticleFilter::updateGPS(double gps_x, double gps_y, double std_gps[2]) {
    std::normal_distribution<double> gps_noise_x(0, std_gps[0]);
    std::normal_distribution<double> gps_noise_y(0, std_gps[1]);
    double sum_weights = 0.0;
    for (auto& p : particles_) {
        double dx = p.x - gps_x + gps_noise_x(gen_);
        double dy = p.y - gps_y + gps_noise_y(gen_);
        // Gaussian likelihood
        double weight = std::exp(-0.5 * (dx * dx / (std_gps[0] * std_gps[0]) +
                                         dy * dy / (std_gps[1] * std_gps[1])));
        p.weight = weight;
        sum_weights += weight;
    }
    // Normalize weights
    for (auto& p : particles_) p.weight /= sum_weights;

    // Resample
    std::vector<Particle> new_particles;
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    double step = 1.0 / particles_.size();
    double beta = 0.0;
    size_t index = size_t(dist(gen_) * particles_.size());
    double mw = std::max_element(particles_.begin(), particles_.end(),
                                 [](const Particle& a, const Particle& b) {
                                     return a.weight < b.weight;
                                 })
                    ->weight;
    for (size_t i = 0; i < particles_.size(); ++i) {
        beta += dist(gen_) * 2.0 * mw;
        while (beta > particles_[index].weight) {
            beta -= particles_[index].weight;
            index = (index + 1) % particles_.size();
        }
        new_particles.push_back(particles_[index]);
    }
    particles_ = std::move(new_particles);
}

Vec2 ParticleFilter::getPosition() const {
    double x = 0.0, y = 0.0;
    for (const auto& p : particles_) {
        x += p.x;
        y += p.y;
    }
    return Vec2{x / particles_.size(), y / particles_.size()};
}

double ParticleFilter::getOrientation() const {
    double sin_sum = 0.0, cos_sum = 0.0;
    for (const auto& p : particles_) {
        sin_sum += std::sin(p.theta);
        cos_sum += std::cos(p.theta);
    }
    return std::atan2(sin_sum / particles_.size(), cos_sum / particles_.size());
}

std::vector<Particle> ParticleFilter::getParticles() const {
    return particles_;
}