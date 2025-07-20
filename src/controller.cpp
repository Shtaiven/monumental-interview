#include "controller.h"

#include <cmath>

robot_client::Input controller(const robot_model::RobotModel &model,
                               const Vec2 &setpoint) {
    // TODO: Implement controller

    // First rotate the robot to the target theta
    auto current_theta = model.getOrientation();
    double target_theta = std::atan2(setpoint.y, setpoint.x + 1e-6);
    double theta_diff = target_theta - current_theta;
    if (std::abs(theta_diff) > 1e-3) {
        // Rotate towards target
        double angular_velocity = std::min(2.0, std::abs(theta_diff) * 10.0);
        if (theta_diff < 0) {
            angular_velocity = -angular_velocity;  // Rotate left
        }
        std::cout << "Rotating " << theta_diff << " radians" << std::endl;
        return robot_client::Input{.v_left = -angular_velocity,
                                   .v_right = angular_velocity};
    }

    // Next move towards the setpoint
    auto current_pos = model.getPosition();
    Vec2 distance_vec{setpoint.x - current_pos.x, setpoint.y - current_pos.y};
    double distance = std::sqrt(distance_vec.x * distance_vec.x +
                                distance_vec.y * distance_vec.y);
    if (distance > 1e-6) {
        // Move towards target
        std::cout << "Moving " << distance << " meters towards target"
                  << std::endl;
        double linear_velocity =
            std::min(2.0, distance * 10.0);  // Cap speed at 2 m/s
        return robot_client::Input{.v_left = linear_velocity,
                                   .v_right = linear_velocity};
    }

    // Don't move if we're close to the setpoint
    return robot_client::Input{.v_left = 0.0, .v_right = 0.0};
}
