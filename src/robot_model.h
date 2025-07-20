#pragma once

#include <optional>

#include "robot_client.h"
#include "types.h"

namespace robot_model {

class RobotModel {
   public:
    RobotModel() = default;
    ~RobotModel() = default;

    // Add methods to update the robot's state, compute kinematics, etc.
    void update(double dt,
                std::optional<robot_client::Sensors> sensors = std::nullopt);
    void setVelocity(double v_left, double v_right) {
        v_left_ = v_left;
        v_right_ = v_right;
    }
    Point getPosition() { return pos_; }
    double getOrientation() { return theta_; }

   private:
    Point pos_{0.0, 0.0};   // Robot's x position
    double theta_ = 0.0;    // Robot's orientation angle
    double v_left_ = 0.0;   // Left wheel velocity
    double v_right_ = 0.0;  // Right wheel velocity
};

}  // namespace robot_model
