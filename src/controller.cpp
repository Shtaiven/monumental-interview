#include "controller.h"

#include <cmath>

// PD gains
#define K_P_THETA 0.1
#define K_D_THETA 0.01
#define K_P_POSITION 1.0
#define K_D_POSITION 0.2

robot_client::Input controller(const robot_model::RobotModel &model,
                               const Vec2 &setpoint) {
    static double last_theta_diff = 0.0;
    static double last_distance = 0.0;
    static double last_time = 0.0;

    auto current_pos = model.getPosition();
    Vec2 distance_vec{setpoint.x - current_pos.x, setpoint.y - current_pos.y};
    double distance = std::sqrt(distance_vec.x * distance_vec.x +
                                distance_vec.y * distance_vec.y);

    double current_time = model.getLifetimeSeconds();
    double dt = (last_time > 0.0) ? (current_time - last_time) : 0.05;

    // Stop if close enough
    if (distance <= 0.25) {
        last_theta_diff = 0.0;
        last_distance = 0.0;
        last_time = current_time;
        return {0, 0};
    }

    double current_theta = model.getOrientation();
    double target_theta = std::atan2(distance_vec.y, distance_vec.x + 1e-6);
    double theta_diff = target_theta - current_theta;

    // Normalize theta_diff to [-pi, pi]
    while (theta_diff > M_PI) theta_diff -= 2 * M_PI;
    while (theta_diff < -M_PI) theta_diff += 2 * M_PI;

    double d_theta = (dt > 0.0) ? (theta_diff - last_theta_diff) / dt : 0.0;
    double d_distance = (dt > 0.0) ? (distance - last_distance) / dt : 0.0;

    // PD control for both
    double linear_velocity =
        K_P_POSITION * distance + K_D_POSITION * d_distance;
    double angular_velocity = K_P_THETA * theta_diff + K_D_THETA * d_theta;

    // Clamp velocities
    // linear_velocity = std::max(0.0, std::min(2.0, linear_velocity));
    // angular_velocity = std::max(-2.0, std::min(2.0, angular_velocity));

    // Differential drive mixing
    double v_left = linear_velocity - angular_velocity;
    double v_right = linear_velocity + angular_velocity;

    // Clamp wheel velocities
    v_left = std::max(-2.0, std::min(2.0, v_left));
    v_right = std::max(-2.0, std::min(2.0, v_right));

    last_theta_diff = theta_diff;
    last_distance = distance;
    last_time = current_time;

    return robot_client::Input{.v_left = v_left, .v_right = v_right};
}
