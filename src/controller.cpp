#include "controller.h"

#include <cmath>

robot_client::Input controller(const robot_model::RobotModel &model,
                               const Vec2 &setpoint) {
    // Inverse kinematics for a differential drive mobile robot is difficult,
    // Simplest is to first rotate towards the setpoint, then drive forward
    // towards it
    auto current_pos = model.getPosition();
    Vec2 distance_vec{setpoint.x - current_pos.x, setpoint.y - current_pos.y};
    double distance = std::sqrt(distance_vec.x * distance_vec.x +
                                distance_vec.y * distance_vec.y);

    // If we're already close to the setpoint, stop!
    if (distance <= 0.005) {
        return {0, 0};
    }

    // First rotate the robot to the target theta
    auto current_theta = model.getOrientation();
    double target_theta = std::atan2(
        distance_vec.y,
        distance_vec.x + 1e-6);  // add a small diff to prevent divide by 0
    double theta_diff = target_theta - current_theta;
    if (std::abs(theta_diff) > 0.05) {
        // Rotate towards target
        double angular_velocity = std::max(
            0.0, std::min(2.0, std::abs(theta_diff) * 1.0));  // clamp [0, 2]
        if (theta_diff < 0) {
            angular_velocity = -angular_velocity;  // Rotate left
        }
        std::cout << "[DEBUG] Rotating " << theta_diff << " radians"
                  << std::endl;
        return robot_client::Input{.v_left = -angular_velocity,
                                   .v_right = angular_velocity};
    }

    // Next move towards the setpoint
    std::cout << "[DEBUG] Moving " << distance << " meters towards target"
              << std::endl;
    double linear_velocity =
        std::max(0.0, std::min(2.0, distance * 1.0));  // clamp [0, 2]
    return robot_client::Input{.v_left = linear_velocity,
                               .v_right = linear_velocity};
}
