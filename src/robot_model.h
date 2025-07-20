#pragma once

#include <cmath>
#include <optional>

#include "robot_client.h"
#include "types.h"

namespace robot_model {

class RobotModel {
   public:
    RobotModel() = default;
    ~RobotModel() = default;

    // Add methods to update the robot's state, compute kinematics, etc.
    void update(const robot_client::Sensors &sensors);
    void setVelocity(double v_left, double v_right) {
        v_left_ = v_left;
        v_right_ = v_right;
    }
    Vec2 getPosition() const { return pos_; }
    double getOrientation() const { return theta_; }
    double getLifetimeSeconds() const;

   private:
    void computePosition();

    const double axle_length_ = 0.5;  // The distance between the center of each
                                      // of the robot's wheels (m)

    Vec2 pos_{0.0, 0.0};            // Robot's position predicted by model (m)
    Vec2 gps_pos_{0.0, 0.0};        // Robot's position according to gps (m)
    double theta_ = 0.0;            // Robot's orientation angle (rad)
    Vec2 linear_acc_ = {0.0, 0.0};  // Linear acceleration (m/s^2)
    double angular_vel_ = 0.0;      // Angular velocity (rad/s)
    double v_left_ = 0.0;           // Velocity of the left wheel (m/s)
    double v_right_ = 0.0;          // Velocity of the right wheel (m/s)
    std::optional<int64_t> start_time_ =
        std::nullopt;  // start time for the lifetime computation
    std::optional<int64_t> linear_acc_time_ =
        std::nullopt;  // Timestamp for linear acceleration
    std::optional<int64_t> last_linear_acc_time_ =
        std::nullopt;  // Timestamp for last linear acceleration
    std::optional<int64_t> angular_vel_time_ =
        std::nullopt;  // Timestamp for angular velocity
    std::optional<int64_t> last_angular_vel_time_ =
        std::nullopt;  // Timestamp for last angular velocity
    std::optional<int64_t> gps_pos_time_ =
        std::nullopt;  // Timestamp for GPS data
    std::optional<int64_t> last_gps_pos_time_ =
        std::nullopt;  // Timestamp for last GPS data
};

}  // namespace robot_model
