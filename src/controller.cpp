#include "controller.h"

#include <cmath>

// PD gains
#define K_P_THETA 0.5
#define K_D_THETA 0.1
#define K_P_POSITION 1.5
#define K_D_POSITION 0.1

robot_client::Input controller(const robot_model::RobotModel &model,
                               const Vec2 &setpoint) {
    static double last_theta_diff = 0.0;
    static double last_distance = 0.0;
    static double last_time = 0.0;

    auto current_pos = model.getPosition();
    Vec2 distance_vec{setpoint.x - current_pos.x, setpoint.y - current_pos.y};
    double distance = std::sqrt(distance_vec.x * distance_vec.x +
                                distance_vec.y * distance_vec.y);

    // Get current time (seconds)
    double current_time = model.getLifetimeSeconds();
    double dt = (last_time > 0.0) ? (current_time - last_time)
                                  : 0.05;  // fallback to 0.05s

    // If we're already close to the setpoint, stop!
    if (distance <= 0.25) {
        last_theta_diff = 0.0;
        last_distance = 0.0;
        last_time = current_time;
        return {0, 0};
    }

    // First rotate the robot to the target theta
    auto current_theta = model.getOrientation();
    double target_theta = std::atan2(distance_vec.y, distance_vec.x + 1e-6);
    double theta_diff = target_theta - current_theta;

    // Derivative terms
    double d_theta = (dt > 0.0) ? (theta_diff - last_theta_diff) / dt : 0.0;
    double d_distance = (dt > 0.0) ? (distance - last_distance) / dt : 0.0;

    if (std::abs(theta_diff) > 0.1) {
        // PD control for rotation
        double angular_velocity = K_P_THETA * theta_diff + K_D_THETA * d_theta;
        angular_velocity =
            std::max(-2.0, std::min(2.0, angular_velocity));  // clamp [-2, 2]
        std::cout << "[DEBUG] Rotating " << theta_diff << " radians"
                  << " (diff=" << d_theta << ")" << std::endl;
        last_theta_diff = theta_diff;
        last_distance = distance;
        last_time = current_time;
        return robot_client::Input{.v_left = -angular_velocity,
                                   .v_right = angular_velocity};
    }

    // PD control for forward motion
    double linear_velocity =
        K_P_POSITION * distance + K_D_POSITION * d_distance;
    linear_velocity =
        std::max(0.0, std::min(2.0, linear_velocity));  // clamp [0, 2]
    std::cout << "[DEBUG] Moving " << distance << " meters towards target"
              << " (diff=" << d_distance << ")" << std::endl;

    last_theta_diff = theta_diff;
    last_distance = distance;
    last_time = current_time;
    return robot_client::Input{.v_left = linear_velocity,
                               .v_right = linear_velocity};
}
